#include "video_thread.h"
#include <QDebug>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

VideoThread::VideoThread(QObject *parent)
    : QThread(parent), running(true), newFrameAvailable(false) {
}

VideoThread::~VideoThread() {
    running = false;
    wait();
}

QImage VideoThread::getCurrentFrame() {
    QMutexLocker locker(&frameMutex);
    newFrameAvailable = false;
    return currentFrame.copy();
}

bool VideoThread::hasNewFrame() {
    return newFrameAvailable;
}

void VideoThread::run() {
    AVFormatContext *formatCtx = nullptr;
    AVCodecContext *codecCtx = nullptr;
    const AVCodec *codec = nullptr;
    AVFrame *frame = nullptr;
    AVFrame *frameRGB = nullptr;
    AVPacket packet;
    struct SwsContext *swsCtx = nullptr;
    uint8_t *buffer = nullptr;
    
    // Initialize FFmpeg (av_register_all is deprecated in FFmpeg 4.0+)
    avformat_network_init();
    
    formatCtx = avformat_alloc_context();
    
    // Set options for low latency
    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "fflags", "nobuffer", 0);
    av_dict_set(&opts, "flags", "low_delay", 0);
    av_dict_set(&opts, "probesize", "32", 0);
    av_dict_set(&opts, "analyzeduration", "0", 0);
    
    // Open TCP stream
    if (avformat_open_input(&formatCtx, "tcp://127.0.0.1:12341", nullptr, &opts) != 0) {
        qWarning() << "Failed to open video stream";
        av_dict_free(&opts);
        return;
    }
    av_dict_free(&opts);
    
    if (avformat_find_stream_info(formatCtx, nullptr) < 0) {
        qWarning() << "Failed to find stream info";
        avformat_close_input(&formatCtx);
        return;
    }
    
    // Find video stream
    int videoStream = -1;
    for (unsigned i = 0; i < formatCtx->nb_streams; i++) {
        if (formatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStream = i;
            break;
        }
    }
    
    if (videoStream == -1) {
        qWarning() << "No video stream found";
        avformat_close_input(&formatCtx);
        return;
    }
    
    // Get codec
    AVCodecParameters *codecParams = formatCtx->streams[videoStream]->codecpar;
    codec = avcodec_find_decoder(codecParams->codec_id);
    if (!codec) {
        qWarning() << "Codec not found";
        avformat_close_input(&formatCtx);
        return;
    }
    
    codecCtx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codecCtx, codecParams);
    
    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        qWarning() << "Could not open codec";
        avcodec_free_context(&codecCtx);
        avformat_close_input(&formatCtx);
        return;
    }
    
    // Allocate frames
    frame = av_frame_alloc();
    frameRGB = av_frame_alloc();
    
    int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, codecCtx->width, codecCtx->height, 1);
    buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
    av_image_fill_arrays(frameRGB->data, frameRGB->linesize, buffer, AV_PIX_FMT_RGB24, codecCtx->width, codecCtx->height, 1);
    
    swsCtx = sws_getContext(codecCtx->width, codecCtx->height, codecCtx->pix_fmt,
                            codecCtx->width, codecCtx->height, AV_PIX_FMT_RGB24,
                            SWS_BILINEAR, nullptr, nullptr, nullptr);
    
    qDebug() << "Video stream opened:" << codecCtx->width << "x" << codecCtx->height;
    
    // Read frames
    while (running && av_read_frame(formatCtx, &packet) >= 0) {
        if (packet.stream_index == videoStream) {
            if (avcodec_send_packet(codecCtx, &packet) == 0) {
                while (avcodec_receive_frame(codecCtx, frame) == 0) {
                    // Convert to RGB
                    sws_scale(swsCtx, frame->data, frame->linesize, 0, codecCtx->height,
                              frameRGB->data, frameRGB->linesize);
                    
                    // Create QImage
                    QImage img(frameRGB->data[0], codecCtx->width, codecCtx->height,
                               frameRGB->linesize[0], QImage::Format_RGB888);
                    
                    QMutexLocker locker(&frameMutex);
                    currentFrame = img.copy();
                    newFrameAvailable = true;
                    locker.unlock();
                    
                    emit frameReady();
                }
            }
        }
        av_packet_unref(&packet);
    }
    
    // Cleanup
    av_free(buffer);
    av_frame_free(&frameRGB);
    av_frame_free(&frame);
    sws_freeContext(swsCtx);
    avcodec_free_context(&codecCtx);
    avformat_close_input(&formatCtx);
    avformat_network_deinit();
}
