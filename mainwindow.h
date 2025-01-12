#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtNetwork/QTcpSocket>
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLegendMarker>
#include <QToolTip>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <windows.h>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <qdatetime.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket;
    QTimer *updateTimer;
    bool isRunning;
    
    // Screen dimensions
    int xmax;
    int ymax;
    
    // Current state
    int currentLevel;
    double currentX;
    double currentY;
    double targetX;
    double targetY;
    double distanceToTarget;
    
    // Chart components
    QChart *chart;
    QLineSeries *positionErrorXSeries;
    QLineSeries *positionErrorYSeries;
    QLineSeries *velocityXSeries;
    QLineSeries *velocityYSeries;
    QLineSeries *controlSignalXSeries;
    QLineSeries *controlSignalYSeries;
    QValueAxis *timeAxis;
    QValueAxis *errorAxis;
    QValueAxis *velocityAxis;
    QValueAxis *controlAxis;

    bool lastGameStopFlag = true;
    
    // Add time tracking
    double t = 0.0;  // Current time in seconds
    QTime startTime;  // To track when we started

    // Add these declarations
    void updateAxisRanges();
    qint64 chartStartTime;

    // Add these declarations:
    QTimer* dataTimeoutTimer;
    static const int DATA_TIMEOUT_MS = 1000; // 1 second timeout

    // Methods
    void setupUI();
    void setupCharts();
    void adjustAxisRange(QValueAxis* axis, double minVal, double maxVal, bool isSymmetric = false);
    void moveMouseAbsolute(double x, double y);
    void processGameData(const QJsonObject &data);
    QString getControlUnitForLevel(int level);

    void solveLevel0();
    void solveLevel1(const QJsonObject &data);
    void solveLevel2_MAX(const QJsonObject &data);
    void solveLevel2_PD(const QJsonObject &data);
    void solveLevel3(const QJsonObject &data);
    void solveLevel4(const QJsonObject &data);

    // void solveGeneralizedLevel(const QJsonObject &data);
    double getSeriesMinY(QLineSeries* series);
    double getSeriesMaxY(QLineSeries* series);
    void logMessage(const QString& category, const QString& message, const QColor& color = Qt::white);

    double getNiceNumber(double value, bool round = true);

    void setupGUI();

    void updateChartData(double timeSeconds, double errorX, double errorY, double velocityX, double velocityY, double controlX, double controlY);

    void saveResults(const QString& timestamp, int level, double targetX, double targetY, double ttime);
    void saveChart(const QString& timestamp, int level, double ttime);
    double currentLevelTime;

    void clickScreenCenter();

private slots:
    void connectToServer();
    void disconnectFromServer();
    void handleSocketData();
    void handleSocketError(QAbstractSocket::SocketError error);
    void startSolver();
    void stopSolver();
    void handleDataTimeout();
    void handleLegendDoubleClick(QLegendMarker* marker);

};
#endif // MAINWINDOW_H
