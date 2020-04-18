#pragma once

#include <string>
#include <vector>
#include <tuple>

#include <Eigen/Core>

extern "C"
{
#include <libavutil/imgutils.h>
#include <libavutil/error.h>
#include <libavutil/samplefmt.h>
#include <libavutil/timestamp.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

#include "gpmf-parser/GPMF_parser.h"
}

class Media
{
public:
    enum PacketType
    {
        IMU,
        VIDEO,
        OTHER,
        END,
    };


    Media(std::string path);
    ~Media();

    void open();

    enum Media::PacketType next_packet();

    enum PacketType get_packet_type() { return packet_type; }
    double get_pts() { return pts; }
    double get_duration() { return duration; }

    bool read_imu(std::vector<Eigen::Vector3f>& accl_data, std::vector<Eigen::Vector3f>& gyro_data);
    bool read_frame();

    void set_frame_width(int frame_width) { this->frame_width = frame_width; }
    void set_frame_height(int frame_height) { this->frame_height = frame_height; }
    int get_frame_linesize() { return frame_rgb->linesize[0]; }
    int get_frame_height() { return frame_rgb->height; }
    int get_frame_width() { return frame_rgb->width; }
    const uint8_t * get_frame_data() { return frame_rgb->data[0]; }

    void reset();

    // int extract_imu(std::vector<std::pair<uint64_t, std::vector<Eigen::Vector3f>>> &accl_data, std::vector<std::pair<uint64_t, std::vector<Eigen::Vector3f>>> &gyro_data);
    // int read_frame(int *got_frame);

    // AVFrame *frameRGB = NULL;
    // int64_t duration;
    // int64_t pos;
    // int64_t dts;
    // int64_t pts;

    // AVStream *video_stream = NULL;
    // AVStream *gpmf_stream = NULL;


protected:
    int open_codec_context(int *stream_idx,
                           AVCodecContext **dec_ctx, AVFormatContext *fmt_ctx, enum AVMediaType type);

    // int decode_packet(int *got_frame, int cached);
    // void debug_gpmf(GPMF_stream *ms);

private:
    std::string path;

    // ffmpeg related
    AVFormatContext *fmt_ctx = NULL;
    AVCodecContext *video_dec_ctx = NULL;

    int video_stream_idx = -1;
    int gpmf_stream_idx = 3;

    AVStream *video_stream = nullptr;
    AVStream *gpmf_stream = nullptr;

    int width, height;
    enum AVPixelFormat pix_fmt;
    uint8_t *video_dst_data[4] = {NULL};
    int video_dst_linesize[4];
    int video_dst_bufsize;

    AVFrame *frame = nullptr;
    AVPacket pkt;

    AVFrame *frame_rgb = nullptr;
    int frame_width  = -1;
    int frame_height = -1;

    // SWS
    SwsContext *img_convert_ctx = nullptr;


    int64_t start_bytepos = -1;

    enum PacketType packet_type;
    double pts;
    double duration;

    // int video_stream_idx = -1, audio_stream_idx = -1;
    // int gpmf_stream_idx = 3;

    // int video_frame_count = 0;
    // int audio_frame_count = 0;

    // // SWS
    // SwsContext *imgConvertCtx = nullptr;
};
