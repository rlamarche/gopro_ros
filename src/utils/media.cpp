#include "media.h"

#include "ros/ros.h"

#define ERROR_BUF_SIZE 256

#define DBG_MSG printf

#define VERBOSE_OUTPUT 0

#if VERBOSE_OUTPUT
#define LIMITOUTPUT arraysize = structsize;
#else
#define LIMITOUTPUT                  \
    if (arraysize > 1 && repeat > 3) \
        repeat = 3, dots = 1;        \
    else if (repeat > 6)             \
        repeat = 6, dots = 1;
#endif

static void debug_gpmf(GPMF_stream *ms);
static void printfData(uint32_t type, uint32_t structsize, uint32_t repeat, void *data);

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

    if (img_convert_ctx != nullptr)
    {
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

    char type;
    void *data;
    uint32_t *L;

    do
    {

        switch (GPMF_Key(ms))
        {
        // case STR2FOURCC("TSMP"):
        //     type = GPMF_Type(ms);
        //     data = GPMF_RawData(ms);
        //     L = (uint32_t *)data;
        //     ROS_INFO("TSMP = %d", BYTESWAP32(*L));

        //     break;
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

    if (frame_width < 0)
        frame_width = width;
    if (frame_height < 0)
        frame_height = height;

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
        }
        else
        {
            return false;
        }
    }

    return decoded;
}

static void debug_gpmf(GPMF_stream *ms)
{
    if (ms)
    {
        uint32_t key = GPMF_Key(ms);
        uint32_t type = GPMF_Type(ms);
        uint32_t structsize = GPMF_StructSize(ms);
        uint32_t repeat = GPMF_Repeat(ms);
        uint32_t size = GPMF_RawDataSize(ms);
        uint32_t indent, level = GPMF_NestLevel(ms);
        void *data = GPMF_RawData(ms);

        if (key != GPMF_KEY_DEVICE)
            level++;

        indent = level;
        while (indent > 0 && indent < 10)
        {
            DBG_MSG("  ");
            indent--;
        }
        if (type == 0)
            DBG_MSG("%c%c%c%c nest size %d ", (key >> 0) & 0xff, (key >> 8) & 0xff, (key >> 16) & 0xff, (key >> 24) & 0xff, size);
        else if (structsize == 1 || (repeat == 1 && type != '?'))
            DBG_MSG("%c%c%c%c type '%c' size %d ", (key >> 0) & 0xff, (key >> 8) & 0xff, (key >> 16) & 0xff, (key >> 24) & 0xff, type == 0 ? '0' : type, size);
        else
            DBG_MSG("%c%c%c%c type '%c' samplesize %d repeat %d ", (key >> 0) & 0xff, (key >> 8) & 0xff, (key >> 16) & 0xff, (key >> 24) & 0xff, type == 0 ? '0' : type, structsize, repeat);

        if (type && repeat > 0)
        {
            DBG_MSG("data: ");

            if (type == GPMF_TYPE_COMPLEX)
            {
                GPMF_stream find_stream;
                GPMF_CopyState(ms, &find_stream);
                if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, GPMF_CURRENT_LEVEL))
                {
                    char *srctype = (char *)GPMF_RawData(&find_stream);
                    uint32_t typelen = GPMF_RawDataSize(&find_stream);
                    int struct_size_of_type;

                    struct_size_of_type = GPMF_SizeOfComplexTYPE(srctype, typelen);
                    if (struct_size_of_type != (int32_t)structsize)
                    {
                        DBG_MSG("error: found structure of %d bytes reported as %d bytes", struct_size_of_type, structsize);
                    }
                    else
                    {
                        char typearray[64];
                        uint32_t elements = sizeof(typearray);
                        uint8_t *bdata = (uint8_t *)data;
                        uint32_t i;

                        if (GPMF_OK == GPMF_ExpandComplexTYPE(srctype, typelen, typearray, &elements))
                        {
                            uint32_t j;
#if !VERBOSE_OUTPUT
                            if (repeat > 3)
                                repeat = 3;
#endif
                            for (j = 0; j < repeat; j++)
                            {
                                if (repeat > 1)
                                {
                                    DBG_MSG("\n  ");

                                    indent = level;
                                    while (indent > 0 && indent < 10)
                                    {
                                        DBG_MSG("  ");
                                        indent--;
                                    }
                                }
                                for (i = 0; i < elements; i++)
                                {
                                    int elementsize = (int)GPMF_SizeofType((GPMF_SampleType)typearray[i]);
                                    printfData(typearray[i], elementsize, 1, bdata);
                                    bdata += elementsize;
                                }
                            }
                            if (repeat > 1)
                                DBG_MSG("...");
                        }
                    }
                }
                else
                {
                    DBG_MSG("unknown formatting");
                }
            }
            else
            {
                printfData(type, structsize, repeat, data);
            }
        }

        DBG_MSG("\n");
    }
}

static void printfData(uint32_t type, uint32_t structsize, uint32_t repeat, void *data)
{
    int dots = 0;

    switch (type)
    {
    case GPMF_TYPE_STRING_ASCII:
    {
        char t[256];
        int size = structsize * repeat;
        uint32_t arraysize = structsize;
        LIMITOUTPUT;

        if (size > 255)
        {
            size = 255;
        }
        memcpy(t, (char *)data, size);
        t[size] = 0;

        if (arraysize == 1 || repeat == 1)
        {
            DBG_MSG("\"%s\"", t);
            dots = 0;
        }
        else
        {
            uint32_t i, j, pos = 0;
            for (i = 0; i < repeat; i++)
            {
                DBG_MSG("\"");
                for (j = 0; j < arraysize; j++)
                {
                    if (t[pos] != '\0' && t[pos] != ' ')
                        DBG_MSG("%c", t[pos]);
                    pos++;
                }
                DBG_MSG("\", ");
            }
        }
    }
    break;
    case GPMF_TYPE_SIGNED_BYTE:
    {
        int8_t *b = (int8_t *)data;
        uint32_t arraysize = structsize;
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize;

            while (arraysize--)
            {
                DBG_MSG("%d,", (int8_t)*b);
                b++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_UNSIGNED_BYTE:
    {
        uint8_t *b = (uint8_t *)data;
        uint32_t arraysize = structsize;
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize;

            while (arraysize--)
            {
                DBG_MSG("%d,", *b);
                b++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_DOUBLE:
    {
        uint64_t Swap, *L = (uint64_t *)data;
        double *d;
        uint32_t arraysize = structsize / sizeof(uint64_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(uint64_t);

            while (arraysize--)
            {
                Swap = BYTESWAP64(*L);
                d = (double *)&Swap;
                DBG_MSG("%.3f,", *d);
                L++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_FLOAT:
    {
        uint32_t Swap, *L = (uint32_t *)data;
        float *f;
        uint32_t arraysize = structsize / sizeof(uint32_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(uint32_t);

            while (arraysize--)
            {
                Swap = BYTESWAP32(*L);
                f = (float *)&Swap;
                DBG_MSG("%.3f,", *f);
                L++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_FOURCC:
    {
        uint32_t *L = (uint32_t *)data;
        uint32_t arraysize = structsize / sizeof(uint32_t);
        LIMITOUTPUT;
        while (repeat--)
        {
            arraysize = structsize / sizeof(uint32_t);

            while (arraysize--)
            {
                DBG_MSG("%c%c%c%c,", PRINTF_4CC(*L));
                L++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_GUID: // display GUID in this formatting ABCDEF01-02030405-06070809-10111213
    {
        uint8_t *B = (uint8_t *)data;
        uint32_t arraysize = structsize;
        LIMITOUTPUT;
        while (repeat--)
        {
            arraysize = structsize;

            while (arraysize--)
            {
                DBG_MSG("%02X", *B);
                B++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_SIGNED_SHORT:
    {
        int16_t *s = (int16_t *)data;
        uint32_t arraysize = structsize / sizeof(int16_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(int16_t);

            while (arraysize--)
            {
                DBG_MSG("%d,", (int16_t)BYTESWAP16(*s));
                s++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_UNSIGNED_SHORT:
    {
        uint16_t *S = (uint16_t *)data;
        uint32_t arraysize = structsize / sizeof(uint16_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(uint16_t);

            while (arraysize--)
            {
                DBG_MSG("%d,", BYTESWAP16(*S));
                S++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_SIGNED_LONG:
    {
        int32_t *l = (int32_t *)data;
        uint32_t arraysize = structsize / sizeof(uint32_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(uint32_t);

            while (arraysize--)
            {
                DBG_MSG("%d,", (int32_t)BYTESWAP32(*l));
                l++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_UNSIGNED_LONG:
    {
        uint32_t *L = (uint32_t *)data;
        uint32_t arraysize = structsize / sizeof(uint32_t);
        LIMITOUTPUT;
        while (repeat--)
        {
            arraysize = structsize / sizeof(uint32_t);

            while (arraysize--)
            {
                DBG_MSG("%d,", BYTESWAP32(*L));
                L++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_Q15_16_FIXED_POINT:
    {
        int32_t *q = (int32_t *)data;
        uint32_t arraysize = structsize / sizeof(int32_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(int32_t);

            while (arraysize--)
            {
                double dq = BYTESWAP32(*q);
                dq /= (double)65536.0;
                DBG_MSG("%.3f,", dq);
                q++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_Q31_32_FIXED_POINT:
    {
        int64_t *Q = (int64_t *)data;
        uint32_t arraysize = structsize / sizeof(int64_t);
        LIMITOUTPUT;

        while (repeat--)
        {
            arraysize = structsize / sizeof(int64_t);

            while (arraysize--)
            {
                uint64_t Q64 = (uint64_t)BYTESWAP64(*Q);
                double dq = (double)(Q64 >> (uint64_t)32);
                dq += (double)(Q64 & (uint64_t)0xffffffff) / (double)0x100000000;
                DBG_MSG("%.3f,", dq);
                Q++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_UTC_DATE_TIME:
    {
        char *U = (char *)data;
        uint32_t arraysize = structsize / 16;
        LIMITOUTPUT;
        while (repeat--)
        {
            arraysize = structsize / 16;
            char t[17];
            t[16] = 0;

            while (arraysize--)
            {
#ifdef _WINDOWS
                strncpy_s(t, 17, U, 16);
#else
                strncpy(t, U, 16);
#endif
                DBG_MSG("\"%s\",", t);
                U += 16;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_SIGNED_64BIT_INT:
    {
        int64_t *J = (int64_t *)data;
        uint32_t arraysize = structsize / sizeof(int64_t);
        LIMITOUTPUT;
        while (repeat--)
        {
            arraysize = structsize / sizeof(int64_t);

            while (arraysize--)
            {
                DBG_MSG("%lld,", (long long int)BYTESWAP64(*J));
                J++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    case GPMF_TYPE_UNSIGNED_64BIT_INT:
    {
        uint64_t *J = (uint64_t *)data;
        uint32_t arraysize = structsize / sizeof(uint64_t);
        LIMITOUTPUT;
        while (repeat--)
        {
            arraysize = structsize / sizeof(uint64_t);

            while (arraysize--)
            {
                DBG_MSG("%llu,", (long long unsigned int)BYTESWAP64(*J));
                J++;
            }
            if (repeat)
                DBG_MSG(" ");
        }
    }
    break;
    default:
        break;
    }

    if (dots) // more data was not output
        DBG_MSG("...");
}
