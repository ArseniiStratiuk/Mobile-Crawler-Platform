#include "main_window.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QDateTime>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Vepryk Control");
    resize(1200, 700);
    
    setupUI();
    applyDarkTheme();
    
    // Update display at 30 FPS
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updateDisplay);
    updateTimer->start(33);
}

MainWindow::~MainWindow() {
}

void MainWindow::setupUI() {
    QWidget *centralWidget = new QWidget(this);
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);
    
    // Left side: Video
    videoDisplay = new VideoDisplay(this);
    
    // Right side: Controls
    QVBoxLayout *controlsLayout = new QVBoxLayout();
    
    // Status section
    QGroupBox *statusBox = new QGroupBox("Connection Status");
    QVBoxLayout *statusLayout = new QVBoxLayout();
    statusLabel = new QLabel("Disconnected");
    statusLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: red;");
    armLabel = new QLabel("DISARMED");
    armLabel->setStyleSheet("font-size: 14px; font-weight: bold; color: orange;");
    statusLayout->addWidget(statusLabel);
    statusLayout->addWidget(armLabel);
    statusBox->setLayout(statusLayout);
    
    // Control section with visual indicators
    QGroupBox *controlBox = new QGroupBox("Control Status");
    QHBoxLayout *controlLayout = new QHBoxLayout();
    
    // Throttle column (vertical indicator)
    QVBoxLayout *throttleLayout = new QVBoxLayout();
    throttleLabel = new QLabel("Throttle: 1500");
    throttleLabel->setAlignment(Qt::AlignCenter);
    throttleIndicator = new ControlIndicator(ControlIndicator::Vertical);
    throttleLayout->addWidget(throttleLabel);
    throttleLayout->addWidget(throttleIndicator);
    
    // Gear in center
    QVBoxLayout *gearLayout = new QVBoxLayout();
    gearLayout->addStretch();
    gearLabel = new QLabel("Gear: Unknown");
    gearLabel->setStyleSheet("font-size: 36px; font-weight: bold;");
    gearLabel->setAlignment(Qt::AlignCenter);
    gearLayout->addWidget(gearLabel);
    gearLayout->addStretch();
    
    // Steering column (horizontal indicator)
    QVBoxLayout *steeringLayout = new QVBoxLayout();
    steeringLabel = new QLabel("Steering: 1500");
    steeringLabel->setAlignment(Qt::AlignCenter);
    steeringIndicator = new ControlIndicator(ControlIndicator::Horizontal);
    steeringLayout->addWidget(steeringLabel);
    steeringLayout->addWidget(steeringIndicator);
    
    controlLayout->addLayout(throttleLayout);
    controlLayout->addLayout(gearLayout);
    controlLayout->addLayout(steeringLayout);
    controlBox->setLayout(controlLayout);
    
    // Telemetry section (empty but structured)
    QGroupBox *telemetryBox = new QGroupBox("Telemetry");
    QVBoxLayout *telemetryLayout = new QVBoxLayout();
    batteryLabel = new QLabel("");
    speedLabel = new QLabel("");
    attitudeLabel = new QLabel("");
    telemetryLayout->addWidget(batteryLabel);
    telemetryLayout->addWidget(speedLabel);
    telemetryLayout->addWidget(attitudeLabel);
    telemetryBox->setLayout(telemetryLayout);
    
    // Add sections to controls layout
    controlsLayout->addWidget(statusBox);
    controlsLayout->addWidget(controlBox);
    controlsLayout->addWidget(telemetryBox);
    controlsLayout->addStretch();
    
    // Add video and controls to main layout
    mainLayout->addWidget(videoDisplay, 3);     // Video takes 3/4 width
    mainLayout->addLayout(controlsLayout, 1);   // Controls take 1/4 width
    
    setCentralWidget(centralWidget);
}

void MainWindow::applyDarkTheme() {
    setStyleSheet(R"(
        QMainWindow {
            background-color: #2b2b2b;
        }
        QGroupBox {
            color: #ffffff;
            font-size: 14px;
            font-weight: bold;
            border: 2px solid #555555;
            border-radius: 5px;
            margin-top: 10px;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }
        QLabel {
            color: #ffffff;
            font-size: 12px;
            padding: 5px;
        }
    )");
}

void MainWindow::updateDisplay() {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Update connection status
    uint64_t now = QDateTime::currentMSecsSinceEpoch();
    bool isConnected = telemetryData.connected && 
                      (now - telemetryData.last_heartbeat_ms < 3000);
    
    if (isConnected) {
        statusLabel->setText(QString("Connected (ID: %1:%2)")
            .arg(telemetryData.system_id)
            .arg(telemetryData.component_id));
        statusLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: green;");
    } else {
        statusLabel->setText("Disconnected");
        statusLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: red;");
    }
    
    // Update arm status
    if (telemetryData.armed) {
        armLabel->setText("ARMED");
        armLabel->setStyleSheet("font-size: 14px; font-weight: bold; color: red;");
    } else {
        armLabel->setText("DISARMED");
        armLabel->setStyleSheet("font-size: 14px; font-weight: bold; color: orange;");
    }
    
    // Update control values
    throttleLabel->setText(QString("Throttle: %1").arg(telemetryData.throttle));
    throttleIndicator->setValue(telemetryData.throttle);
    
    // Invert steering for display (config inverts for MAVLink, we invert back for GUI)
    uint16_t displaySteering = 3000 - telemetryData.steering; // 1000<->2000, 1500 stays 1500
    steeringLabel->setText(QString("Steering: %1").arg(telemetryData.steering));
    steeringIndicator->setValue(displaySteering);
    
    // Display actual gear from ESTIMATOR_STATUS (bigger font)
    QString gearText;
    if (telemetryData.gear_known) {
        if (telemetryData.actual_gear == 0.0f) {
            gearText = "N";
        } else {
            gearText = QString::number(telemetryData.actual_gear, 'f', 1);
        }
    } else {
        gearText = "?";
    }
    gearLabel->setText(gearText);
    
    // Telemetry section - empty for now
    batteryLabel->setText("");
    speedLabel->setText("");
    attitudeLabel->setText("");
}
