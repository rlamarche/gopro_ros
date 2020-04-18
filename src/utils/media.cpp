#include "media.h"

#include "ros/ros.h"

#define ERROR_BUF_SIZE 256

Media::Media(std::string path) : path(path)
{
}

Media::~Media()
{
    if (frame_rgb != nullptr)
    {
        av_frame_unref(frame_rgb);
        frame_rgb = nullptr;
    }
    avcodec_free_context(&video_dec_ctx);
    avformat_close_input(&fmt_ctx);
    av_frame_free(&frame);
    av_free(video_dst_data[0]);

    if (img_convert_ctx != nullptr) {
        sws_freeContext(img_convert_ctx);
        img_convert_ctx = nullptr;
    }
}

void Media::open()
{
    int ret = 0;

    /* open input file, and allocate format context */
    ROS_INFO("Open file %s", path.c_str());
    int ret_avformat_open_input = avformat_open_input(&fmt_ctx, path.c_str(), NULL, NULL);
    if (ret_avformat_open_input < 0)
    {
        char buffer[ERROR_BUF_SIZE];
        av_make_error_string(buffer, ERROR_BUF_SIZE, ret_avformat_open_input);
        ROS_ERROR("Could not open source file %s, error: %s", path.c_str(), buffer);
        throw std::logic_error("Could not open media.");
    }

    /* retrieve stream information */
    if (avformat_find_stream_info(fmt_ctx, NULL) < 0)
    {
        ROS_ERROR("Could not find stream information.");
        throw std::logic_error("Could not find stream information.");
    }

    open_codec_context(&video_stream_idx, &video_dec_ctx, fmt_ctx, AVMEDIA_TYPE_VIDEO);

    video_stream = fmt_ctx->streams[video_stream_idx];
    gpmf_stream = fmt_ctx->streams[gpmf_stream_idx];

    ROS_INFO("Video stream time base %d/%d", video_stream->time_base.num, video_stream->time_base.den);
    ROS_INFO("GPMF stream time base %d/%d", gpmf_stream->time_base.num, gpmf_stream->time_base.den);

    /* allocate image where the decoded image will be put */
    width = video_dec_ctx->width;
    height = video_dec_ctx->height;
    pix_fmt = video_dec_ctx->pix_fmt;
    ret = av_image_alloc(video_dst_data, video_dst_linesize,
                         width, height, pix_fmt, 1);
    if (ret < 0)
    {
        ROS_ERROR("Could not allocate raw video buffer.");
        throw std::logic_error("Could not allocate raw video buffer.");
    }

    video_dst_bufsize = ret;

    char pix_fmt_str[256];
    av_get_pix_fmt_string(pix_fmt_str, 256, pix_fmt);
    ROS_INFO("Found pixel format: %s", pix_fmt_str);

    /* dump input information to stderr */
    av_dump_format(fmt_ctx, 0, path.c_str(), 0);

    frame = av_frame_alloc();
    if (!frame)
    {
        ROS_ERROR("Could not allocate frame.");
        ret = AVERROR(ENOMEM);
        throw std::logic_error("Could not allocate frame.");
    }

    /* initialize packet, set data to NULL, let the demuxer fill it */
    av_init_packet(&pkt);
    pkt.data = NULL;
    pkt.size = 0;

    if (video_stream)
    {
        ROS_INFO("Demuxing video from file '%s'", path.c_str());
    }
}

enum Media::PacketType Media::next_packet()
{
    av_packet_unref(&pkt);

    int ret = 0, got_frame = 0;
    int64_t bytepos = -1;

    ret = av_read_frame(fmt_ctx, &pkt);
    if (ret != 0)
    {
        ROS_INFO("got packet type END");
        packet_type = PacketType::END;
        return packet_type;
    }

    if (start_bytepos < 0)
    {
        start_bytepos = pkt.pos;
    }

    pts = pkt.pts;
    duration = pkt.duration;

    if (pkt.stream_index == gpmf_stream_idx)
    {
        packet_type = PacketType::IMU;
        pts *= av_q2d(gpmf_stream->time_base);
        duration *= av_q2d(gpmf_stream->time_base);
        // ROS_INFO("got packet type IMU ts = %f duration = %f", pts, duration);
    }
    else if (pkt.stream_index == video_stream_idx)
    {
        packet_type = PacketType::VIDEO;
        pts *= av_q2d(video_stream->time_base);
        duration *= av_q2d(video_stream->time_base);
        // ROS_INFO("got packet type VIDEO ts = %f duration = %f", pts, duration);
    }
    else
    {
        // ROS_INFO("got packet type OTHER");
        packet_type = PacketType::OTHER;
    }

    return packet_type;
}

bool Media::read_imu(std::vector<Eigen::Vector3f> &accl_data, std::vector<Eigen::Vector3f> &gyro_data)
{
    GPMF_stream metadata_stream, *ms = &metadata_stream;

    /* read frames from the file */
    int ret = GPMF_Init(ms, (uint32_t *)pkt.data, pkt.size);

    if (ret != GPMF_OK)
        return false;

    uint32_t samples;
    float_t temp_buffer[512 * 3];
    uint32_t temp_buffersize = sizeof(float_t) * 512 * 3;

    // do
    // {
    //     debug_gpmf(ms); // printf current GPMF KLV
    // } while (GPMF_OK == GPMF_Next(ms, GPMF_RECURSE_LEVELS));
    // GPMF_ResetState(ms);

    do
    {

        switch (GPMF_Key(ms))
        {
        case STR2FOURCC("ACCL"):
            // Found accelerometer
            samples = GPMF_Repeat(ms);
            // ROS_INFO("Found samples: %d\n", samples);
            if (GPMF_OK == GPMF_ScaledData(ms, temp_buffer, temp_buffersize, 0, samples, GPMF_TYPE_FLOAT))
            { /* Process scaled values */
                for (int i = 0; i < 3 * samples; i += 3)
                {
                    accl_data.push_back(Eigen::Vector3f(temp_buffer[i + 2], temp_buffer[i + 1], temp_buffer[i]));
                }
            }

            break;
        case STR2FOURCC("GYRO"):
            // Found gyro
            samples = GPMF_Repeat(ms);
            // ROS_INFO("Found GYRO samples: %d\n", samples);
            if (GPMF_OK == GPMF_ScaledData(ms, temp_buffer, temp_buffersize, 0, samples, GPMF_TYPE_FLOAT))
            { /* Process scaled values */
                for (int i = 0; i < 3 * samples; i += 3)
                {
                    // ROS_INFO("Push gyro: %f\n", temp_buffer[i]);
                    gyro_data.push_back(Eigen::Vector3f(temp_buffer[i + 2], temp_buffer[i + 1], temp_buffer[i]));
                }
            }

            break;

        default: // if you don’t know the Key you can skip to the next
            break;
        }

        // PrintGPMF(ms); // printf current GPMF KLV
    } while (GPMF_OK == GPMF_Next(ms, GPMF_RECURSE_LEVELS));

    GPMF_ResetState(ms);

    return true;
}

void Media::reset()
{
    int ret = avformat_seek_file(
        fmt_ctx, -1,
        start_bytepos, start_bytepos, start_bytepos,
        AVSEEK_FLAG_ANY);

    if (ret < 0)
    {
        char buffer[ERROR_BUF_SIZE];
        av_make_error_string(buffer, ERROR_BUF_SIZE, ret);
        ROS_ERROR("Unable to reset media: %s", buffer);
        throw std::logic_error("Unable to reset media.");
    }
}

bool Media::read_frame()
{
    int ret = 0;
    int decoded = pkt.size;
    
    if (frame_width < 0) frame_width = width;
    if (frame_height < 0) frame_height = height;

    enum AVPixelFormat target_pix_fmt = AV_PIX_FMT_GRAY8;
    // enum AVPixelFormat target_pix_fmt = AV_PIX_FMT_RGB24;

    int got_frame = 0;

    if (pkt.stream_index == video_stream_idx)
    {
        /* decode video frame */
        ret = avcodec_decode_video2(video_dec_ctx, frame, &got_frame, &pkt);
        if (ret < 0)
        {
            char buffer[ERROR_BUF_SIZE];
            av_make_error_string(buffer, ERROR_BUF_SIZE, ret);
            ROS_ERROR("Error decoding video frame: %s", buffer);
            throw std::logic_error("Error decoding video frame.");
        }

        if (got_frame)
        {
            if (frame->width != width || frame->height != height ||
                frame->format != pix_fmt)
            {
                /* To handle this change, one could call av_image_alloc again and
                 * decode the following frames into another rawvideo file. */
                ROS_ERROR("Error: Width, height and pixel format have to be "
                          "constant in a rawvideo file, but the width, height or "
                          "pixel format of the input video changed:\n"
                          "old: width = %d, height = %d, format = %s\n"
                          "new: width = %d, height = %d, format = %s\n",
                          width, height, av_get_pix_fmt_name(pix_fmt),
                          frame->width, frame->height,
                          av_get_pix_fmt_name((AVPixelFormat)frame->format));

                throw std::logic_error("Error: Width, height and pixel format have to be constant.");
            }

            /* copy decoded frame to destination buffer:
             * this is required since rawvideo expects non aligned data */
            av_image_copy(video_dst_data, video_dst_linesize,
                          (const uint8_t **)(frame->data), frame->linesize,
                          pix_fmt, width, height);

            /* write to rawvideo file */
            // fwrite(video_dst_data[0], 1, video_dst_bufsize, video_dst_file);

            if (frame_rgb == nullptr)
            {
                frame_rgb = av_frame_alloc();
                //Allocate memory for the pixels of a picture and setup the AVPicture fields for it.
                // avpicture_alloc((AVPicture *)frameRGB, target_pix_fmt, target_width, target_height);
                av_image_alloc(frame_rgb->data, frame_rgb->linesize, frame_width, frame_height, target_pix_fmt, 1);
            }
            if (!frame_rgb)
            {
                ROS_ERROR("Could not allocate video frame RGB.");
                throw std::logic_error("Could not allocate video frame RGB.");
            }

            if (img_convert_ctx == nullptr)
            {
                // qDebug() << "Image size:" << video_dec_ctx->width << video_dec_ctx->height;
                img_convert_ctx = sws_getContext(video_dec_ctx->width, video_dec_ctx->height, video_dec_ctx->pix_fmt, frame_width, frame_height, target_pix_fmt, SWS_BICUBIC, NULL, NULL, NULL);
            }

            sws_scale(img_convert_ctx, frame->data, frame->linesize, 0, frame->height, frame_rgb->data, frame_rgb->linesize);
            frame_rgb->width = frame_width;
            frame_rgb->height = frame_height;

            // do something with the frame in rgb

            av_frame_unref(frame);
        } else {
            return false;
        }
    }

    return decoded;
}
