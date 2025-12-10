#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QWidget>
#include <QPainter>
#include <mutex>
#include "video_display.h"

struct TelemetryData {
    bool connected = false;
    bool armed = false;
    uint8_t system_id = 0;
    uint8_t component_id = 0;
    
    // RC channels
    uint16_t throttle = 0;
    uint16_t steering = 0;
    uint16_t gear = 0;
    
    // Actual gear state from ESTIMATOR_STATUS
    float actual_gear = 0.0f;
    bool gear_known = false;
    
    // Battery
    float voltage = 0.0f;
    int8_t battery_remaining = -1;
    
    // Attitude
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    
    // Position/Speed
    float groundspeed = 0.0f;
    
    uint64_t last_heartbeat_ms = 0;
};

// Visual indicator for throttle/steering
class ControlIndicator : public QWidget {
    Q_OBJECT
public:
    enum Orientation { Vertical, Horizontal };
    
    ControlIndicator(Orientation orient, QWidget *parent = nullptr)
        : QWidget(parent), orientation(orient), value(1500) {
        setMinimumSize(orient == Vertical ? QSize(60, 200) : QSize(200, 60));
    }
    
    void setValue(uint16_t val) {
        value = val;
        update();
    }
    
protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // Draw background
        painter.fillRect(rect(), QColor(60, 60, 60));
        
        // Draw center line
        painter.setPen(QPen(QColor(100, 100, 100), 2));
        if (orientation == Vertical) {
            // Vertical line for throttle
            int centerX = width() / 2;
            painter.drawLine(centerX, 0, centerX, height());
        } else {
            // Horizontal line for steering (kept as is)
            int centerY = height() / 2;
            painter.drawLine(0, centerY, width(), centerY);
        }
        
        // Calculate position (1000-2000 range)
        float normalized = (value - 1000.0f) / 1000.0f; // 0.0 to 1.0
        normalized = qBound(0.0f, normalized, 1.0f);
        
        int dotX, dotY;
        int dotRadius = 8;
        if (orientation == Vertical) {
            // Throttle: dot moves vertically, top edge at top when 2000
            dotX = width() / 2;
            dotY = height() - (int)(normalized * (height() - 2 * dotRadius)) - dotRadius;
        } else {
            // Steering: dot moves horizontally
            dotX = (int)(normalized * (width() - 2 * dotRadius)) + dotRadius;
            dotY = height() / 2;
        }
        
        // Draw dot
        painter.setBrush(QColor(0, 255, 0));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPoint(dotX, dotY), dotRadius, dotRadius);
    }
    
private:
    Orientation orientation;
    uint16_t value;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    TelemetryData telemetryData;
    std::mutex dataMutex;

private slots:
    void updateDisplay();

private:
    void setupUI();
    void applyDarkTheme();
    
    QLabel *statusLabel;
    QLabel *armLabel;
    QLabel *throttleLabel;
    QLabel *steeringLabel;
    QLabel *gearLabel;
    QLabel *batteryLabel;
    QLabel *attitudeLabel;
    QLabel *speedLabel;
    
    ControlIndicator *throttleIndicator;
    ControlIndicator *steeringIndicator;
    
    VideoDisplay *videoDisplay;
    
    QTimer *updateTimer;
};

#endif
