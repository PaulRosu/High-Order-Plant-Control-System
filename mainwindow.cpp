#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QCursor>
#include <QScreen>
#include <QDateTime>
#include <QtCharts>
#include <QtCharts/QValueAxis>
#include <QCoreApplication>
#include <deque>
#include <QColorDialog>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QXYLegendMarker>
#include <QKeyEvent>
#include <chrono>
#include <QDir>
#include <QFile>
#include <QTextStream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , socket(new QTcpSocket(this))
    , isRunning(false)
{
    ui->setupUi(this);
    setupUI();
    setupCharts();

    // Get screen dimensions
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    xmax = screenGeometry.width();
    ymax = screenGeometry.height();

    // Connect socket signals
    connect(socket, &QTcpSocket::readyRead, this, &MainWindow::handleSocketData);
    connect(socket, &QTcpSocket::errorOccurred, this, &MainWindow::handleSocketError);

    // Connect UI signals
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::connectToServer);
    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::startSolver);
    connect(ui->stopButton, &QPushButton::clicked, this, &MainWindow::stopSolver);

    // Setup data timeout timer
    dataTimeoutTimer = new QTimer(this);
    dataTimeoutTimer->setSingleShot(true);
    dataTimeoutTimer->setInterval(DATA_TIMEOUT_MS);
    connect(dataTimeoutTimer, &QTimer::timeout, this, &MainWindow::handleDataTimeout);

    // In MainWindow constructor or in main.cpp after QApplication creation
    setupGUI();
}

MainWindow::~MainWindow()
{
    if (socket->state() == QAbstractSocket::ConnectedState) {
        socket->disconnectFromHost();
    }
    delete ui;
}

void MainWindow::setupUI()
{
    // Initialize UI state
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(false);

    // Setup log text properties
    ui->logText->setLineWrapMode(QTextEdit::NoWrap);
    ui->logText->document()->setMaximumBlockCount(1000); // Limit number of log lines
    connectToServer();
}

void MainWindow::setupCharts()
{
    // Create chart
    chart = new QChart();

    // Create series for different metrics
    positionErrorXSeries = new QLineSeries();
    positionErrorYSeries = new QLineSeries();
    velocityXSeries = new QLineSeries();
    velocityYSeries = new QLineSeries();
    controlSignalXSeries = new QLineSeries();
    controlSignalYSeries = new QLineSeries();

    // Set series names
    positionErrorXSeries->setName("Error X [pixels]");
    positionErrorYSeries->setName("Error Y [pixels]");
    velocityXSeries->setName("Velocity X [pixels/s]");
    velocityYSeries->setName("Velocity Y [pixels/s]");
    controlSignalXSeries->setName("Control X");
    controlSignalYSeries->setName("Control Y");

    // Add series to chart
    chart->addSeries(positionErrorXSeries);
    chart->addSeries(positionErrorYSeries);
    chart->addSeries(velocityXSeries);
    chart->addSeries(velocityYSeries);
    chart->addSeries(controlSignalXSeries);
    chart->addSeries(controlSignalYSeries);

    // Create axes
    timeAxis = new QValueAxis();
    errorAxis = new QValueAxis();
    velocityAxis = new QValueAxis();
    controlAxis = new QValueAxis();

    // Setup axes with initial ranges
    timeAxis->setTitleText("Time [s]");
    timeAxis->setRange(0, 5);  // Start with 5-second window
    timeAxis->setTickCount(6);
    timeAxis->setMinorTickCount(1);
    timeAxis->setGridLineVisible(false);  // Hide vertical grid lines
    timeAxis->setMinorGridLineVisible(false);  // Hide minor vertical grid lines

    // Configure value axes to only show zero grid line
    errorAxis->setTitleText("Error [pixels]");
    errorAxis->setRange(-20, 20);  // Initial range with negative values
    errorAxis->setTickCount(6);
    errorAxis->setTitleBrush(positionErrorXSeries->color());
    errorAxis->setLabelsBrush(positionErrorXSeries->color());
    //errorAxis->setGridLineVisible(false);  // Hide all grid lines
    errorAxis->setMinorGridLineVisible(false);  // Hide minor grid lines
    // Add zero line with higher visibility
    QPen zeroPen(QColor(128, 128, 128));  // Lighter gray for better visibility
    zeroPen.setStyle(Qt::SolidLine);
    zeroPen.setWidth(1);  // Make the line slightly thicker
    errorAxis->setLinePen(zeroPen);
    errorAxis->setLineVisible(true);

    velocityAxis->setTitleText("Velocity [pixels/s]");
    velocityAxis->setRange(-20, 20);  // Initial range with negative values
    velocityAxis->setTickCount(6);
    velocityAxis->setTitleBrush(velocityXSeries->color());
    velocityAxis->setLabelsBrush(velocityXSeries->color());
    velocityAxis->setGridLineVisible(false);  // Hide all grid lines
    velocityAxis->setMinorGridLineVisible(false);  // Hide minor grid lines
    // Add zero line
    velocityAxis->setLinePen(zeroPen);
    velocityAxis->setLineVisible(true);

    controlAxis->setTitleText("Control");
    controlAxis->setRange(-1, 1);
    controlAxis->setTickCount(5);
    controlAxis->setGridLineVisible(false);  // Hide all grid lines
    controlAxis->setMinorGridLineVisible(false);  // Hide minor grid lines
    // Add zero line
    controlAxis->setLinePen(zeroPen);
    controlAxis->setLineVisible(true);

    // Add axes to chart
    chart->addAxis(timeAxis, Qt::AlignBottom);
    chart->addAxis(errorAxis, Qt::AlignLeft);
    chart->addAxis(velocityAxis, Qt::AlignLeft);
    chart->addAxis(controlAxis, Qt::AlignRight);

    // Attach series to axes
    positionErrorXSeries->attachAxis(timeAxis);
    positionErrorXSeries->attachAxis(errorAxis);
    positionErrorYSeries->attachAxis(timeAxis);
    positionErrorYSeries->attachAxis(errorAxis);

    velocityXSeries->attachAxis(timeAxis);
    velocityXSeries->attachAxis(velocityAxis);
    velocityYSeries->attachAxis(timeAxis);
    velocityYSeries->attachAxis(velocityAxis);

    controlSignalXSeries->attachAxis(timeAxis);
    controlSignalXSeries->attachAxis(controlAxis);
    controlSignalYSeries->attachAxis(timeAxis);
    controlSignalYSeries->attachAxis(controlAxis);

    // Chart appearance
    chart->setTheme(QChart::ChartThemeDark);

    // Set series colors with warm (X) and cool (Y) colors
    // Error - Most saturated
    positionErrorXSeries->setColor(QColor(255, 0, 0));  
    positionErrorYSeries->setColor(QColor(170, 0, 127));   

    // Velocity - Medium saturation
    velocityXSeries->setColor(QColor(255, 150, 30));  
    velocityYSeries->setColor(QColor(255, 255, 0));   

    // Control - Less saturated
    controlSignalXSeries->setColor(QColor(0, 170, 0));
    controlSignalYSeries->setColor(QColor(0, 170, 255));

    chart->setBackgroundBrush(QColor(32, 32, 32));
    chart->setTitleBrush(QColor(255, 255, 255));
    chart->legend()->setAlignment(Qt::AlignBottom);
    
    // Update legend marker labels to match series colors
    const auto markers = chart->legend()->markers();
    for (QLegendMarker* marker : markers) {
        if (auto* lineMarker = qobject_cast<QXYLegendMarker*>(marker)) {
            QLineSeries* series = qobject_cast<QLineSeries*>(lineMarker->series());
            if (series) {
                marker->setLabelBrush(series->color());
            }
        }
    }

    // Set the chart on the view
    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);

    // Connect legend markers to double-click handler
    for (QLegendMarker* marker : markers) {
        connect(marker, &QLegendMarker::clicked, this, [this, marker]() {
            handleLegendDoubleClick(marker);
        });
    }
}

void MainWindow::handleLegendDoubleClick(QLegendMarker* marker)
{
    if (auto* lineMarker = qobject_cast<QXYLegendMarker*>(marker)) {
        QLineSeries* series = qobject_cast<QLineSeries*>(lineMarker->series());
        if (series) {
            // Get current color
            QColor currentColor = series->color();
            
            // Open color dialog
            QColor newColor = QColorDialog::getColor(currentColor, this, 
                "Choose Series Color", QColorDialog::ShowAlphaChannel);
            
            if (newColor.isValid()) {
                series->setColor(newColor);
                // Update the legend marker color
                marker->setLabelBrush(newColor);
            }
        }
    }
}

void MainWindow::connectToServer()
{
    if (socket->state() == QAbstractSocket::UnconnectedState) {
        qDebug() << "Connecting to server...";
        socket->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
        socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

        connect(socket, &QTcpSocket::connected, this, [this]() {
            qDebug() << "Connected successfully";
            ui->logText->append(QString("[%1] Connected to server").arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
            ui->connectButton->setText("Disconnect");
            ui->startButton->setEnabled(true);
            // startSolver();
        });

        socket->connectToHost("localhost", 30000);

        if (!socket->waitForConnected(5000)) {
            qDebug() << "Failed to connect to server";
            qDebug() << "Error:" << socket->error() << "-" << socket->errorString();
            ui->logText->append(QString("[%1] Connection failed: %2")
                                    .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
                                    .arg(socket->errorString()));
        }
    } else {
        qDebug() << "Disconnecting from server...";
        disconnectFromServer();
    }
}

void MainWindow::disconnectFromServer()
{
    socket->disconnectFromHost();
    ui->connectButton->setText("Connect");
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(false);
    ui->logText->append(QString("[%1] Disconnected from server").arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
}

void MainWindow::handleSocketData()
{
    QByteArray data = socket->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);

    if (!doc.isNull() && doc.isObject()) {
        processGameData(doc.object());
    }
}

void MainWindow::handleSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg = QString("[%1] Socket error: %2")
    .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
        .arg(socket->errorString());
    ui->logText->append(errorMsg);

    if (error == QAbstractSocket::RemoteHostClosedError) {
        disconnectFromServer();
    }
}


void MainWindow::startSolver()
{
    isRunning = true;
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(true);
    ui->logText->append(QString("[%1] Solver started").arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
    
    // Clear all series data
    positionErrorXSeries->clear();
    positionErrorYSeries->clear();
    velocityXSeries->clear();
    velocityYSeries->clear();
    controlSignalXSeries->clear();
    controlSignalYSeries->clear();
    
    // Reset time reference to exactly now
    chartStartTime = QDateTime::currentMSecsSinceEpoch();
    
    // Reset axis ranges
    timeAxis->setRange(0, 5);
    errorAxis->setRange(-20, 20);
    velocityAxis->setRange(-20, 20);
    controlAxis->setRange(-1, 1);

    // Add initial points at t=0
    double dx = targetX - currentX;
    double dy = targetY - currentY;
    positionErrorXSeries->append(0, dx);
    positionErrorYSeries->append(0, dy);
    velocityXSeries->append(0, 0);
    velocityYSeries->append(0, 0);
    controlSignalXSeries->append(0, 0);
    controlSignalYSeries->append(0, 0);
}

void MainWindow::stopSolver()
{
    isRunning = false;
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(false);
    ui->logText->append(QString("[%1] Solver stopped").arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
}

void MainWindow::processGameData(const QJsonObject &data)
{
    // Add static variables for timing
    static std::chrono::steady_clock::time_point lastCall = std::chrono::steady_clock::now();
    static std::deque<double> deltaTimes(10, 0.0);  // Initialize with 10 zeros
    static double movingAverage = 0.0;
    
    // Calculate time delta
    auto now = std::chrono::steady_clock::now();
    auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(now - lastCall).count() / 1000.0; // Convert to milliseconds
    lastCall = now;
    
    // Update moving average
    movingAverage -= deltaTimes.front() / 10.0;  // Remove oldest value's contribution
    movingAverage += deltaTime / 10.0;           // Add new value's contribution
    deltaTimes.pop_front();                      // Remove oldest value
    deltaTimes.push_back(deltaTime);             // Add new value
    
    // Print timing information
    // qDebug() << "Delta time:" << deltaTime << "ms, Moving average:" << movingAverage << "ms";

    // Reset the timeout timer whenever we receive data
    dataTimeoutTimer->start();

    // Get stopflag from data
    bool stopFlag = data["stopflag"].toBool();
    
    // Handle new level start
    if (lastGameStopFlag && !stopFlag) {
        qDebug() << "New level started!";
        ui->logText->append(QString("[%1] New level started").arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
        
        // Only auto-start charts/tracking if solver is already active
        if (isRunning) {
            // Reset charts and time tracking
            positionErrorXSeries->clear();
            positionErrorYSeries->clear();
            velocityXSeries->clear();
            velocityYSeries->clear();
            controlSignalXSeries->clear();
            controlSignalYSeries->clear();
            chartStartTime = QDateTime::currentMSecsSinceEpoch();
            
            // Reset axis ranges
            timeAxis->setRange(0, 5);  // Start with 5-second window
            errorAxis->setRange(-20, 20);
            velocityAxis->setRange(-20, 20);
            controlAxis->setRange(-1, 1);

            startSolver();

            // Add initial points at t=0
            double dx = targetX - currentX;
            double dy = targetY - currentY;
            updateChartData(0, dx, dy, 0, 0, 0, 0);
        }
    }
    
    lastGameStopFlag = stopFlag;

    // Update current state
    currentLevel = data["Level"].toInt() - 1;
    
    // Update control axis title and series names
    QString controlUnit = getControlUnitForLevel(currentLevel);
    controlAxis->setTitleText(QString("Control [%1]").arg(controlUnit));
    controlSignalXSeries->setName(QString("Control X [%1]").arg(controlUnit));
    controlSignalYSeries->setName(QString("Control Y [%1]").arg(controlUnit));
    
    // Update position state
    currentX = data["x"].toDouble();
    currentY = data["y"].toDouble();
    targetX = data["xfin"].toDouble();
    targetY = data["yfin"].toDouble();

    // Calculate distance to target
    double dx = targetX - currentX;
    double dy = targetY - currentY;
    distanceToTarget = sqrt(dx * dx + dy * dy);

    // Process solver logic if running
    if (isRunning) {
        if (stopFlag) {
            stopSolver();
            return;
        }

        // Choose appropriate solver
        switch (currentLevel) {
        case 0:
            solveLevel0();
            break;
        case 1:
            solveLevel1(data);
            break;
        case 2:
            solveLevel2_PD(data);
            break;
        case 3:
            solveLevel3(data);
            break;
        case 4:
            solveLevel4(data);
            break;
        default:
            break;
        }
    }

    // Get level time
    currentLevelTime = data["ttime"].toDouble();
}

void MainWindow::updateAxisRanges() {
    // Only update if we're running and have data
    if (!isRunning || (positionErrorXSeries->count() == 0 && velocityXSeries->count() == 0)) {
        return;
    }

    // Update error axis range
    if (positionErrorXSeries->count() > 0 || positionErrorYSeries->count() > 0) {
        double maxError = qMax(getSeriesMaxY(positionErrorXSeries), getSeriesMaxY(positionErrorYSeries));
        double minError = qMin(getSeriesMinY(positionErrorXSeries), getSeriesMinY(positionErrorYSeries));
        
        // Ensure we have some range even if values are close to zero
        maxError = qMax(maxError, 10.0);
        minError = qMin(minError, -maxError * 0.25);
        
        double niceMax = getNiceNumber(maxError * 1.2, false);
        double niceMin = getNiceNumber(minError * 1.2, false);
        
        // Always update if the range is significantly different
        if (abs(errorAxis->max() - niceMax) > 1.0 || 
            abs(errorAxis->min() - niceMin) > 1.0) {
            errorAxis->setRange(niceMin, niceMax);
            errorAxis->setTickCount(6);
        }
    }

    // Update velocity axis range
    if (velocityXSeries->count() > 0 || velocityYSeries->count() > 0) {
        double maxVel = qMax(getSeriesMaxY(velocityXSeries), getSeriesMaxY(velocityYSeries));
        double minVel = qMin(getSeriesMinY(velocityXSeries), getSeriesMinY(velocityYSeries));
        
        // Ensure we have some range even if values are close to zero
        maxVel = qMax(maxVel, 10.0);
        minVel = qMin(minVel, -maxVel * 0.25);
        
        double niceMax = getNiceNumber(maxVel * 1.2, false);
        double niceMin = getNiceNumber(minVel * 1.2, false);
        
        // Always update if the range is significantly different
        if (abs(velocityAxis->max() - niceMax) > 1.0 || 
            abs(velocityAxis->min() - niceMin) > 1.0) {
            velocityAxis->setRange(niceMin, niceMax);
            velocityAxis->setTickCount(6);
        }
    }

    // Update control axis range with both X and Y series
    if (controlSignalXSeries->count() > 0 || controlSignalYSeries->count() > 0) {
        double minControlX = getSeriesMinY(controlSignalXSeries);
        double maxControlX = getSeriesMaxY(controlSignalXSeries);
        double minControlY = getSeriesMinY(controlSignalYSeries);
        double maxControlY = getSeriesMaxY(controlSignalYSeries);
        
        double minControl = qMin(minControlX, minControlY);
        double maxControl = qMax(maxControlX, maxControlY);
        
        // Find the larger magnitude to make range symmetric around zero
        double maxMagnitude = qMax(qAbs(minControl), qAbs(maxControl));
        maxMagnitude = qMax(maxMagnitude, 10.0); // Ensure some minimum range
        double niceMagnitude = getNiceNumber(maxMagnitude * 1.2, false);
        
        // Always update if the range is significantly different
        if (abs(controlAxis->max() - niceMagnitude) > 1.0) {
            controlAxis->setRange(-niceMagnitude, niceMagnitude);
            controlAxis->setTickCount(5);  // This ensures 0 is in the middle
        }
    }
}






void MainWindow::moveMouseAbsolute(double x, double y)
{
    // Convert from mathematical coordinates (origin at bottom-left)
    // to screen coordinates (origin at top-left)
    int screenY = ymax - static_cast<int>(y);
    QCursor::setPos(QPoint(static_cast<int>(x), screenY));
}


void MainWindow::logMessage(const QString& category, const QString& message, const QColor& color)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString formattedMessage = QString("[%1] [%2] %3")
                                   .arg(timestamp)
                                   .arg(category)
                                   .arg(message);

    ui->logText->setTextColor(color);
    ui->logText->append(formattedMessage);
}

double MainWindow::getSeriesMinY(QLineSeries* series)
{
    if (series->count() == 0) return 0;
    
    double minY = series->at(0).y();
    for (int i = 1; i < series->count(); ++i) {
        minY = qMin(minY, series->at(i).y());
    }
    return minY;
}

double MainWindow::getSeriesMaxY(QLineSeries* series)
{
    if (series->count() == 0) return 0;
    
    double maxY = series->at(0).y();
    for (int i = 1; i < series->count(); ++i) {
        maxY = qMax(maxY, series->at(i).y());
    }
    return maxY;
}

void MainWindow::handleDataTimeout()
{
    if (isRunning) {
        qDebug() << "Level completed (Data timeout)!";
        
        // Generate timestamp
        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
        
        // Save results to CSV
        saveResults(timestamp, currentLevel, targetX, targetY, currentLevelTime);
        
        // Save chart image
        saveChart(timestamp, currentLevel, currentLevelTime);
        
        ui->logText->append(QString("[%1] Level completed - Time: %2s")
            .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
            .arg(currentLevelTime, 0, 'f', 3));
        
        // Reset the lastStopFlag static variable in processGameData
        lastGameStopFlag = true;

        // Only auto-restart if checkbox is checked
        if (ui->autoRestartCheckbox->isChecked()) {
            // Schedule center click after 2 seconds
            QTimer::singleShot(2000, this, &MainWindow::clickScreenCenter);
        }
    }
}

void MainWindow::saveResults(const QString& timestamp, int level, double targetX, double targetY, double ttime)
{
    QString filename = "results.csv";
    QFile file(filename);
    bool fileExists = file.exists();
    
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        qDebug() << "Failed to open results file for writing";
        return;
    }
    
    QTextStream out(&file);
    
    // Write header if file is new
    if (!fileExists) {
        out << "Timestamp,Level,TargetX,TargetY,Time\n";
    }
    
    // Write data
    out << QString("%1,%2,%3,%4,%5\n")
           .arg(timestamp)
           .arg(level)
           .arg(targetX)
           .arg(targetY)
           .arg(ttime);
    
    file.close();
}

void MainWindow::saveChart(const QString& timestamp, int level, double ttime)
{
    // Create charts directory if it doesn't exist
    QDir().mkpath("charts");
    
    // Generate filename with timestamp and level info
    QString filename = QString("charts/chart_%1_level%2_time%3s.png")
                          .arg(timestamp)
                          .arg(level)
                          .arg(ttime, 0, 'f', 3);
    
    // Get the chart view as a pixmap and save it
    QPixmap pixmap = ui->chartView->grab();
    if (!pixmap.save(filename)) {
        qDebug() << "Failed to save chart image:" << filename;
    }
}

QString MainWindow::getControlUnitForLevel(int level) {
    const QStringList controlUnits = {
        "pixels",       // level 0
        "pixels/s",     // level 1
        "pixels/s²",    // level 2
        "pixels/s³",    // level 3
        "pixels/s⁴"     // level 4
    };
    
    if (level >= 0 && level < controlUnits.size()) {
        return controlUnits[level];
    }
    return "pixels";
}

double MainWindow::getNiceNumber(double value, bool round) {
    if (value == 0) return 0;
    
    // Handle negative numbers by processing the absolute value and restoring sign at the end
    bool isNegative = value < 0;
    value = abs(value);

    // Get the exponent
    double exponent = floor(log10(value));
    double fraction = value / pow(10.0, exponent);
    double niceFraction;

    // For our chart axes, we want numbers divisible by 5 or 10
    if (round) {
        if (fraction < 1.5) niceFraction = 1;
        else if (fraction < 3.5) niceFraction = 2.5;
        else if (fraction < 7.5) niceFraction = 5;
        else niceFraction = 10;
    } else {
        if (fraction <= 1) niceFraction = 1;
        else if (fraction <= 2.5) niceFraction = 2.5;
        else if (fraction <= 5) niceFraction = 5;
        else niceFraction = 10;
    }

    double result = niceFraction * pow(10.0, exponent);
    return isNegative ? -result : result;
}

// Helper function to adjust axis range and ticks to get nice numbers
void MainWindow::adjustAxisRange(QValueAxis* axis, double minVal, double maxVal, bool isSymmetric) {
    // qDebug() << "Adjusting axis range. Input min:" << minVal << "max:" << maxVal;
    
    // Handle special cases
    if (minVal == 0 && maxVal == 0) {
        axis->setRange(-10, 10);
        axis->setTickCount(5);
        return;
    }

    // Get nice numbers for min and max
    double niceMin = getNiceNumber(minVal, true);
    double niceMax = getNiceNumber(maxVal, true);
    
    // qDebug() << "Nice numbers - min:" << niceMin << "max:" << niceMax;
    
    // For symmetric ranges (like control axis)
    if (isSymmetric) {
        double maxMagnitude = qMax(abs(niceMin), abs(niceMax)) * 1.2; // Add extra padding for symmetric ranges
        maxMagnitude = getNiceNumber(maxMagnitude, true);
        niceMin = -maxMagnitude;
        niceMax = maxMagnitude;
    }
    
    // Calculate a nice interval (divisible by 5 or 10)
    double range = niceMax - niceMin;
    double roughInterval = range / 5.0; // Start with 6 ticks (5 intervals)
    
    // Handle the interval calculation
    roughInterval = abs(roughInterval); // Ensure positive interval
    double exponent = floor(log10(roughInterval));
    double fraction = roughInterval / pow(10.0, exponent);
    
    // Round to nice intervals
    if (fraction < 1.5) fraction = 1;
    else if (fraction < 3.5) fraction = 2.5;
    else if (fraction < 7.5) fraction = 5;
    else fraction = 10;
    
    double niceInterval = fraction * pow(10.0, exponent);
    
    // Adjust min and max to be multiples of the nice interval
    // Make sure we extend the range to include the original values
    // niceMin = floor(minVal / niceInterval) * niceInterval;
    // niceMax = ceil(maxVal / niceInterval) * niceInterval;
    
    // Calculate number of tick marks needed
    int tickCount = 5;//round((niceMax - niceMin) / niceInterval) + 1;
    
    // qDebug() << "Final range - min:" << niceMin << "max:" << niceMax << "interval:" << niceInterval << "ticks:" << tickCount;
    
    // Set the range and tick count
    axis->setRange(niceMin, niceMax);
    axis->setTickCount(tickCount);
}

void MainWindow::setupGUI() {
    // Get the default application font
    QFont defaultFont = QApplication::font();
    
    // Increase the font size (e.g., by 2 points)
    defaultFont.setPointSize(defaultFont.pointSize() + 2);
    
    // Apply to entire application
    QApplication::setFont(defaultFont);

    // For chart-specific text, you can set larger fonts for axes and legend
    QFont chartFont = defaultFont;
    chartFont.setPointSize(defaultFont.pointSize() + 1);  // Even larger for charts
    
    // Apply to chart elements
    timeAxis->setTitleFont(chartFont);
    timeAxis->setLabelsFont(chartFont);
    errorAxis->setTitleFont(chartFont);
    errorAxis->setLabelsFont(chartFont);
    velocityAxis->setTitleFont(chartFont);
    velocityAxis->setLabelsFont(chartFont);
    controlAxis->setTitleFont(chartFont);
    controlAxis->setLabelsFont(chartFont);
    
    // Set legend font
    chart->legend()->setFont(chartFont);
}

void MainWindow::updateChartData(double timeSeconds, double errorX, double errorY, double velocityX, double velocityY, double controlX, double controlY) {
    if (!isRunning || timeSeconds < 0) {
        return;
    }

    // Add data points
    positionErrorXSeries->append(timeSeconds, errorX);
    positionErrorYSeries->append(timeSeconds, errorY);
    velocityXSeries->append(timeSeconds, velocityX);
    velocityYSeries->append(timeSeconds, velocityY);
    controlSignalXSeries->append(timeSeconds, controlX);
    controlSignalYSeries->append(timeSeconds, controlY);

    // Update time axis
    timeAxis->setRange(0, timeSeconds + 1);

    // Update ranges based on all historical data
    double maxError = qMax(getSeriesMaxY(positionErrorXSeries), getSeriesMaxY(positionErrorYSeries));
    double minError = qMin(getSeriesMinY(positionErrorXSeries), getSeriesMinY(positionErrorYSeries));
    
    double maxVel = qMax(getSeriesMaxY(velocityXSeries), getSeriesMaxY(velocityYSeries));
    double minVel = qMin(getSeriesMinY(velocityXSeries), getSeriesMinY(velocityYSeries));
    
    // Get control signal range from actual control values
    double maxControlX = getSeriesMaxY(controlSignalXSeries);
    double minControlX = getSeriesMinY(controlSignalXSeries);
    double maxControlY = getSeriesMaxY(controlSignalYSeries);
    double minControlY = getSeriesMinY(controlSignalYSeries);
    
    // Find the overall control range
    double maxControl = qMax(qMax(abs(maxControlX), abs(minControlX)), qMax(abs(maxControlY), abs(minControlY)));

    // Add padding and ensure minimum ranges
    maxError = qMax(maxError * 1.4, 10.0);
    minError = qMin(minError * 1.4, -10.0);
    maxVel = qMax(maxVel * 1.5, 10.0);
    minVel = qMin(minVel * 1.5, -10.0);
    maxControl = qMax(maxControl * 1.5, 1.0); // Use a smaller minimum range for control

    // Adjust ranges with nice numbers
    adjustAxisRange(errorAxis, minError, maxError, true);
    adjustAxisRange(velocityAxis, minVel, maxVel, true);
    adjustAxisRange(controlAxis, -maxControl, maxControl, true);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    // Check if 's' is pressed and solver is running
    if (event->key() == Qt::Key_Space && isRunning) {
        stopSolver();
    }
    
    // Call parent class implementation for other keys
    QMainWindow::keyPressEvent(event);
}






inline double sign(double x) {
    return (x > 0) - (x < 0);
}

void MainWindow::solveLevel0()
{
    // Simply move mouse directly to target position
    moveMouseAbsolute(targetX, targetY);

    QString msg = QString("[%1] Level 0 - Distance: %2, Target: (%3, %4)")
                      .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
                      .arg(distanceToTarget, 0, 'f', 2)
                      .arg(targetX, 0, 'f', 2)
                      .arg(targetY, 0, 'f', 2);
    ui->logText->append(msg);
}

void MainWindow::solveLevel1(const QJsonObject &data)
{
    /* Level 1 Solver - Exact Speed Control
     *
     * Control Strategy:
     * Calculate exact speed needed: distance / timeStep
     */

    // Calculate distances for each component
    double distanceX = targetX - currentX;
    double distanceY = targetY - currentY;

    const double maxControl = 1000.0;
    const double timeStep = 0.1;  // 0.1 seconds per step

    // Calculate control magnitude for each component
    double controlX, controlY;

    // X-axis control
    controlX = abs(distanceX / timeStep);  // Exact speed needed
    controlX = qMin(controlX, maxControl);  // Limit to max control

    // Y-axis control
    controlY = abs(distanceY / timeStep);
    controlY = qMin(controlY, maxControl);

    // Apply direction to control signals
    double controlSignal_X = (distanceX != 0) ? (controlX * distanceX / abs(distanceX)) : 0;
    double controlSignal_Y = (distanceY != 0) ? (controlY * distanceY / abs(distanceY)) : 0;

    // Map to screen coordinates
    double screenPosition_X = xmax/2 + controlSignal_X * (xmax/2) / maxControl;
    double screenPosition_Y = ymax/2 + controlSignal_Y * (ymax/2) / maxControl;

    moveMouseAbsolute(screenPosition_X, screenPosition_Y);

    // Debug logging
    logMessage("SOLVER", QString("Level 1\n"
                                "    Distance(X,Y): (%1, %2)\n"
                                "    Required Speed(X,Y): (%3, %4)\n"
                                "    Control(X,Y): (%5, %6)")
                             .arg(distanceX, 0, 'f', 2)
                             .arg(distanceY, 0, 'f', 2)
                             .arg(controlX, 0, 'f', 2)
                             .arg(controlY, 0, 'f', 2)
                             .arg(controlSignal_X, 0, 'f', 1)
                             .arg(controlSignal_Y, 0, 'f', 1),
               Qt::cyan);
}


// fast version
void MainWindow::solveLevel2_MAX(const QJsonObject &data)
{
    const double dt = 0.1;  // Time step is 0.1 seconds
    const double maxControl = 100.0;
    const double SAFETY_MARGIN = 0.971;

    // Extract current velocity
    QJsonArray velocityXComponents = data["vectorx"].toArray();
    QJsonArray velocityYComponents = data["vectory"].toArray();
    double currentVelocityX = velocityXComponents[1].toDouble();
    double currentVelocityY = velocityYComponents[1].toDouble();

    // Calculate distance to target
    double distanceX = round(targetX - currentX);
    double distanceY = round(targetY - currentY);

    // Check if axes are effectively stopped independently
    const double STOP_SPEED = 2.0;
    const double STOP_DISTANCE = 2.0;
    bool isStoppedX = abs(currentVelocityX) < STOP_SPEED && abs(distanceX) < STOP_DISTANCE;
    bool isStoppedY = abs(currentVelocityY) < STOP_SPEED && abs(distanceY) < STOP_DISTANCE;

    double requiredAccX, requiredAccY;

    // X-axis control with precise acceleration calculation
    if (isStoppedX) {
        requiredAccX = 0.0;
    } else {
        // Calculate stopping distance with current velocity
        double stopDistX = round((currentVelocityX * currentVelocityX) / (2 * maxControl));

        if (abs(stopDistX) >= abs(distanceX) * SAFETY_MARGIN && 
            sign(currentVelocityX) == sign(distanceX)) {
            // Deceleration phase - calculate exact deceleration needed
            double requiredDecel = -(currentVelocityX * currentVelocityX) / (2 * abs(distanceX));
            requiredAccX = qBound(-maxControl, requiredDecel, maxControl) * sign(currentVelocityX);
        } else {
            // Acceleration phase - calculate required acceleration
            double effectiveDistance = distanceX - (currentVelocityX * dt);
            double requiredAcc = effectiveDistance / (0.01); // 0.1 * 0.1 for the two integration steps
            requiredAccX = qBound(-maxControl, requiredAcc, maxControl);
        }
    }

    // Y-axis control (similar logic)
    if (isStoppedY) {
        requiredAccY = 0.0;
    } else {
        double stopDistY = round((currentVelocityY * currentVelocityY) / (2 * maxControl));

        if (abs(stopDistY) >= abs(distanceY) * SAFETY_MARGIN && 
            sign(currentVelocityY) == sign(distanceY)) {
            double requiredDecel = -(currentVelocityY * currentVelocityY) / (2 * abs(distanceY));
            requiredAccY = qBound(-maxControl, requiredDecel, maxControl) * sign(currentVelocityY);
        } else {
            double effectiveDistance = distanceY - (currentVelocityY * dt);
            double requiredAcc = effectiveDistance / (0.01);
            requiredAccY = qBound(-maxControl, requiredAcc, maxControl);
        }
    }

    // Map to screen coordinates
    double screenX = round(xmax/2 + (requiredAccX * (xmax/2)) / maxControl);
    double screenY = round(ymax/2 + (requiredAccY * (ymax/2)) / maxControl);

    moveMouseAbsolute(screenX, screenY);

    // Update chart data
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    double timeSeconds = (currentTime - chartStartTime) / 1000.0;
    updateChartData(timeSeconds, distanceX, distanceY, currentVelocityX, currentVelocityY, requiredAccX, requiredAccY);

    // Debug logging
    logMessage("SOLVER", QString("Level 2\n"
                                "    Distance(X,Y): (%1, %2)\n"
                                "    Current Vel(X,Y): (%4, %5)\n"
                                "    Required Acc(X,Y): (%6, %7)\n"
                                "    Stopped(X,Y): (%8, %9)")
                             .arg(distanceX, 0, 'f', 2)
                             .arg(distanceY, 0, 'f', 2)
                             .arg(currentVelocityX, 0, 'f', 2)
                             .arg(currentVelocityY, 0, 'f', 2)
                             .arg(requiredAccX, 0, 'f', 2)
                             .arg(requiredAccY, 0, 'f', 2)
                             .arg(isStoppedX)
                             .arg(isStoppedY),
               Qt::cyan);
}



// do not delete comments below
void MainWindow::solveLevel2_PD(const QJsonObject &data)
{
    /* Level 2 Solver - PD Control
     *
     * Implements a Proportional-Derivative (PD) controller (P acceleration, D braking):
     *
     * 1. Initial Phase - Acceleration
     *    - Large position error drives strong initial acceleration (P term dominates)
     *    - Minimal braking as velocity is low (D term small)
     *
     * 2. Mid-Phase - Transition
     *    - As velocity increases and error decreases:
     *    - P term naturally reduces
     *    - D term (braking) increases with velocity
     *
     * 3. Final Phase - Stabilization
     *    - P and D terms balance each other
     *    - System settles at target position
     *
     * The control output is mapped to screen coordinates:
     *    - Control signal normalized to ±100 range
     *    - The range is mapped proportionally to screen dimensions
     */

    double KP = 1.0;
    double KD = 1.68;

    // Extract velocity vectors from the game state
    QJsonArray velocityXComponents = data["vectorx"].toArray();
    QJsonArray velocityYComponents = data["vectory"].toArray();

    // Current velocity components for the braking term
    double currentVelocityX = velocityXComponents[1].toDouble();
    double currentVelocityY = velocityYComponents[1].toDouble();

    // Position error drives the acceleration
    double positionErrorX = targetX - currentX;
    double positionErrorY = targetY - currentY;

    // Calculate PD control signals
    // kp * error:     Accelerates toward target (proportional to distance)
    // kd * velocity:  Applies brake (proportional to speed)
    double controlSignalX = KP * positionErrorX - KD * currentVelocityX;
    double controlSignalY = KP * positionErrorY - KD * currentVelocityY;

    // Map control signals to screen coordinates
    // Control signals are normalized to ±100 range (matching MATLAB coeff(2))
    const double maxAcceleration = 100.0;

    // Convert normalized control signals to screen coordinates:
    // - Center point (xmax/2, ymax/2) represents zero control
    // - Scale factor (xmax/2)/maxAcceleration converts control units to pixels
    double screenX = xmax / 2 + controlSignalX * (xmax / 2) / maxAcceleration;
    double screenY = ymax / 2 + controlSignalY * (ymax / 2) / maxAcceleration;

    moveMouseAbsolute(screenX, screenY);

    // Update chart data
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    double timeSeconds = (currentTime - chartStartTime) / 1000.0;
    updateChartData(timeSeconds, positionErrorX, positionErrorY, currentVelocityX, currentVelocityY, controlSignalX, controlSignalY);

    logMessage("SOLVER", QString("Level 2 - Control: (%1, %2)")
                             .arg(controlSignalX, 0, 'f', 2)
                             .arg(controlSignalY, 0, 'f', 2),
               Qt::cyan);
}

void MainWindow::clickScreenCenter()
{
    // Press Enter key
    keybd_event(VK_RETURN, 0, 0, 0);               // Press Enter
    keybd_event(VK_RETURN, 0, KEYEVENTF_KEYUP, 0); // Release Enter
    
    // Wait 500ms
    Sleep(500);
    
    // First TAB
    keybd_event(VK_TAB, 0, 0, 0);               // Press TAB
    keybd_event(VK_TAB, 0, KEYEVENTF_KEYUP, 0); // Release TAB
    
    // Wait 500ms
    Sleep(500);
    
    // Second TAB
    keybd_event(VK_TAB, 0, 0, 0);               // Press TAB
    keybd_event(VK_TAB, 0, KEYEVENTF_KEYUP, 0); // Release TAB
    
    // Wait 500ms
    Sleep(500);
    
    // Press SPACE
    keybd_event(VK_SPACE, 0, 0, 0);               // Press SPACE
    keybd_event(VK_SPACE, 0, KEYEVENTF_KEYUP, 0); // Release SPACE
}


