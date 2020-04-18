#include "media.h"

#include "ros/ros.h"

int Media::open_codec_context(int *stream_idx,
                              AVCodecContext **dec_ctx, AVFormatContext *fmt_ctx, enum AVMediaType type)
{
    int ret, stream_index;
    AVStream *st;
    AVCodec *dec = NULL;
    AVDictionary *opts = NULL;

    ret = av_find_best_stream(fmt_ctx, type, -1, -1, NULL, 0);
    if (ret < 0)
    {
        ROS_ERROR("Could not find %s stream in input file '%s'",
                  av_get_media_type_string(type), path.c_str());
        throw std::logic_error("Could not find %s stream in input file.");
    }
    else
    {
        stream_index = ret;
        st = fmt_ctx->streams[stream_index];

        /* find decoder for the stream */
        dec = avcodec_find_decoder(st->codecpar->codec_id);
        if (!dec)
        {
            ROS_ERROR("Failed to find %s codec\n",
                      av_get_media_type_string(type));
            throw std::logic_error("Failed to find suitable codec.");
        }

        /* Allocate a codec context for the decoder */
        *dec_ctx = avcodec_alloc_context3(dec);
        if (!*dec_ctx)
        {
            ROS_ERROR("Failed to allocate the %s codec context\n",
                      av_get_media_type_string(type));
            throw std::logic_error("Failed to allocate the codec context.");
        }

        /* Copy codec parameters from input stream to output codec context */
        if ((ret = avcodec_parameters_to_context(*dec_ctx, st->codecpar)) < 0)
        {
            ROS_ERROR("Failed to copy %s codec parameters to decoder context\n",
                      av_get_media_type_string(type));
            throw std::logic_error("Failed to copy codec parameters to decoder context.");
        }

        /* Init the decoders, with or without reference counting */
        av_dict_set(&opts, "refcounted_frames", "1", 0);
        if ((ret = avcodec_open2(*dec_ctx, dec, &opts)) < 0)
        {
            ROS_ERROR("Failed to open %s codec\n",
                      av_get_media_type_string(type));
            throw std::logic_error("Failed to open codec.");
        }
        *stream_idx = stream_index;
    }
}
