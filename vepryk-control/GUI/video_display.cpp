#include "video_display.h"
#include <QVBoxLayout>
#include <QPixmap>

VideoDisplay::VideoDisplay(QWidget *parent) : QWidget(parent) {
    setMinimumSize(640, 360);
    
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    videoLabel = new QLabel(this);
    videoLabel->setStyleSheet("background-color: black;");
    videoLabel->setAlignment(Qt::AlignCenter);
    videoLabel->setScaledContents(false);
    layout->addWidget(videoLabel);
    
    // Create and start video thread
    videoThread = new VideoThread(this);
    connect(videoThread, &VideoThread::frameReady, this, &VideoDisplay::updateFrame, Qt::QueuedConnection);
    videoThread->start();
}

VideoDisplay::~VideoDisplay() {
    if (videoThread) {
        videoThread->quit();
        videoThread->wait();
    }
}

void VideoDisplay::updateFrame() {
    if (videoThread->hasNewFrame()) {
        QImage img = videoThread->getCurrentFrame();
        if (!img.isNull()) {
            QPixmap pixmap = QPixmap::fromImage(img);
            // Scale to fit label while keeping aspect ratio
            pixmap = pixmap.scaled(videoLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
            videoLabel->setPixmap(pixmap);
        }
    }
}
