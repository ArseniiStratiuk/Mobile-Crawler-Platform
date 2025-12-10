#ifndef VIDEO_THREAD_H
#define VIDEO_THREAD_H

#include <QThread>
#include <QImage>
#include <QMutex>

class VideoThread : public QThread {
    Q_OBJECT

public:
    VideoThread(QObject *parent = nullptr);
    ~VideoThread();
    
    QImage getCurrentFrame();
    bool hasNewFrame();

signals:
    void frameReady();

protected:
    void run() override;

private:
    volatile bool running;
    QImage currentFrame;
    QMutex frameMutex;
    bool newFrameAvailable;
};

#endif
