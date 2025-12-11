#ifndef VIDEO_DISPLAY_H
#define VIDEO_DISPLAY_H

#include <QWidget>
#include <QLabel>
#include <QImage>
#include "video_thread.h"

class VideoDisplay : public QWidget {
    Q_OBJECT

public:
    VideoDisplay(QWidget *parent = nullptr);
    ~VideoDisplay();

private slots:
    void updateFrame();

private:
    QLabel *videoLabel;
    VideoThread *videoThread;
};

#endif
