#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QtMath>
#include <QDebug>

#include <Eigen/Dense>
#include <Eigen/QR>

/*
Matlab Plant code:
    stopflag = false;
    % params
    coeff = [1,1000,100,10,1,1,1,1,1,1,1];
    % initial position
    PL = get(0,'PointerLocation');
    vectorx = [PL(1),zeros(1,10)];
    vectory = [PL(2),zeros(1,10)];
    % final position
    xfin = xmax*unifrnd(0.1,0.9);
    yfin = ymax*unifrnd(0.1,0.9);

    historyx = PL(1);
    historyy = PL(2);

    TimeLim = 2.5; %s
    thresh = 5; % pix
    Ts = 0.1;
    counter = 0;
    ttime = 0;
    Level = get(handles.LevelSlider,'Value');
    axes(handles.DisplayAxes);
    while(counter < TimeLim && stopflag == false) % keep the dot there for 2.5 second

        PL = get(0,'PointerLocation');

        if(Level == 1)
                x = PL(1);
                y = PL(2);
        else
            vectorx(Level) = (PL(1)-xmax/2) * coeff(Level) / (xmax/2);
            vectory(Level) = (PL(2)-ymax/2) * coeff(Level) / (ymax/2);
            for k = Level-1:-1:1
                vectorx(k) = vectorx(k)+Ts*vectorx(k+1);
                vectory(k) = vectory(k)+Ts*vectory(k+1);
            end
            x = vectorx(1);
            y = vectory(1);
        end

        x = max(11,x); x = min(xmax-10,x);
        y = max(11,y); y = min(ymax-10,y);

        if(x~=vectorx(1))
            vectorx = zeros(size(vectorx));
            vectorx(1) = x;
        end
        if(y~=vectory(1))
            vectory = zeros(size(vectory));
            vectory(1) = y;
        end

        historyx = [historyx,x];
        historyy = [historyy,y];

        % Prepare data to send
        dataStruct.xfin = xfin;
        dataStruct.yfin = yfin;
        dataStruct.x = x;
        dataStruct.y = y;
        dataStruct.vectorx = vectorx;
        dataStruct.vectory = vectory;
        dataStruct.Level = Level;
        dataStruct.Ts = Ts;
        dataStruct.ttime = ttime;
        dataStruct.stopflag = stopflag;

        % Convert to JSON
        jsonStr = jsonencode(dataStruct);
        jsonBytes = uint8(jsonStr);  % Convert to bytes for transmission

        % Send JSON data asynchronously if client is connected
        if tcpServer.Connected
            try
                write(tcpServer, jsonBytes, "uint8");
                flush(tcpServer);
            catch ME
                disp('Error sending data asynchronously over TCP/IP:');
                disp(ME.message);
            end
        end
    .......
    end

*/
void MainWindow::solveLevel3(const QJsonObject &data)
{
    // Time during which the system remains in its 'initial phase' of movement
    const double INITIAL_PHASE_TIME = 0.9;

    // Acceleration and velocity thresholds for the dynamic jerk scaling logic
    const double HIGH_ACC_THRESHOLD = 600.0;
    const double HIGH_VEL_THRESHOLD = 150.0;

    // Elapsed time since this controlling logic started
    double elapsedTime = data["ttime"].toDouble();

    // Check if we are still in the initial phase (where we can be more aggressive)
    bool isInitialPhase = (elapsedTime < INITIAL_PHASE_TIME);

    // Current "Level" of the system, determines the 'coeff' used for scaling
    int Level = currentLevel;

    // Extract arrays from the incoming JSON which hold state variables
    QJsonArray vectorxArray = data["vectorx"].toArray();
    QJsonArray vectoryArray = data["vectory"].toArray();

    // Coefficients for different 'Levels' that scale pointer movement
    double coeff[] = {1, 1000, 100, 10, 1, 1, 1, 1, 1, 1, 1};
    double MAX_INPUT = coeff[Level];

    // Current position (x0,y0), velocity (v0x,v0y), acceleration (a0x,a0y), jerk (j0x,j0y)
    double x0 = vectorxArray[0].toDouble();
    double y0 = vectoryArray[0].toDouble();
    double v0x = vectorxArray[1].toDouble();
    double v0y = vectoryArray[1].toDouble();
    double a0x = vectorxArray[2].toDouble();
    double a0y = vectoryArray[2].toDouble();
    double j0x = vectorxArray[3].toDouble();
    double j0y = vectoryArray[3].toDouble();

    // Desired target position (xf,yf) and final velocity, acceleration targets
    double xf = targetX;
    double yf = targetY;
    double vf = 0.0;  // final desired velocity
    double af = 0.0;  // final desired acceleration

    // Compute the positional error (distance from current position to the target)
    double errorX = xf - x0;
    double errorY = yf - y0;
    double currentError = sqrt(errorX * errorX + errorY * errorY);

    // Absolute velocity and acceleration components, used for dynamic jerk scaling
    double absVx = fabs(v0x);
    double absVy = fabs(v0y);
    double absAx = fabs(a0x);
    double absAy = fabs(a0y);

    // Track the maximum error to normalize how big the current error is, relative to past
    static double maxError = 0.0;
    if (currentError > maxError)
    {
        maxError = currentError;
    }
    else if (isInitialPhase)
    {
        // In the initial phase, we artificially inflate maxError to force larger initial control
        maxError = currentError * 2.5;
    }
    double normalizedError = maxError > 0 ? currentError / maxError : 1.0;

    // Adaptive time horizon 'T', updated as movement progresses
    static double T = 30.0;
    const double MIN_T = 12.0;
    const double MAX_T = 40.0;
    const double ERROR_THRESHOLD = 80.0; // Threshold for error, used to adjust the time horizon

    if (!isInitialPhase)
    {
        // Only recalculate T after the initial phase has ended.
        // The goal is to estimate how long we should plan for,
        // so that the polynomial solver and jerk-based control work properly.

        // Determine direction vector and distance to the target from current position
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = sqrt(dx * dx + dy * dy);

        // If distance is nonzero, compute unit direction
        double dirX = dx / distance;
        double dirY = dy / distance;

        // Project the current velocity and acceleration along the direction to the target
        // This gives us how quickly we are moving toward (or away from) the target
        double vel = v0x * dirX + v0y * dirY;  // velocity component toward the target
        double acc = a0x * dirX + a0y * dirY;  // acceleration component toward the target

        // (1) Compute how long it would take to bring velocity to zero if velocity and
        //     acceleration are in opposite directions. If acc == 0 or not opposing, result is zero.
        double timeToZeroVelocity = (acc != 0 && vel * acc < 0) ? -vel / acc : 0;

        // (2) Estimate how far we'll travel during that 'braking' time (timeToZeroVelocity)
        double distanceAfterBraking = distance - (vel * timeToZeroVelocity +
                                                 0.5 * acc * timeToZeroVelocity * timeToZeroVelocity);

        // (3) Rough estimate of how much time remains to travel the leftover distance
        //     after velocity is presumably zero (or reduced). We use half the current velocity
        //     as a quick guess for the average speed in the remaining leg.
        double timeToTraverseRemaining = distanceAfterBraking / (0.5 * vel);

        // Combine braking time and subsequent travel time for a total naive estimate
        double estimatedTimeToTarget = timeToZeroVelocity + timeToTraverseRemaining;

        // Below, we modulate this time using velocity and error factors:
        // - velocityFactor: the slower we go, the bigger T becomes
        // - errorFactor: the bigger the error, the more we might enlarge T
        double velocityFactorX = qMax(0.0, 1.0 - absVx / HIGH_VEL_THRESHOLD);
        double velocityFactorY = qMax(0.0, 1.0 - absVy / HIGH_VEL_THRESHOLD);
        double velocityFactor = (velocityFactorX + velocityFactorY) / 2.0;

        double errorFactor = qMax(0.0, 1.0 - currentError / ERROR_THRESHOLD);

        // Weighted combination to refine the time horizon estimate
        estimatedTimeToTarget = estimatedTimeToTarget * (0.75 +
                                                         velocityFactor * 0.35 +
                                                         errorFactor * 0.25);

        // Ensure that T remains within specified bounds
        estimatedTimeToTarget = qBound(MIN_T, estimatedTimeToTarget, MAX_T);

        // Finally, we apply exponential smoothing so that T doesn’t jump abruptly:
        //   T_new = (1 - alpha)*T_old + alpha*estimatedTimeToTarget
        double alpha = 0.15;
        T = T * (1 - alpha) + estimatedTimeToTarget * alpha;
    }

    // Powers of T used in the polynomial / jerk-based approach
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    // Ratios used to scale position, velocity, and acceleration inside the polynomial
    double tRatio = T / MAX_T;
    double positionScale = 1.0 / (T3 * tRatio);
    double velocityScale = 1.0 / (T2 * sqrt(tRatio));
    double accelScale = 1.0 / (T * tRatio);

    // Matrix terms factoring in times T3, T4, T5 for position, velocity, acceleration constraints
    double a11 = T3 * positionScale;
    double a12 = T4 * positionScale;
    double a13 = T5 * positionScale;

    double a21 = 3 * T2 * velocityScale;
    double a22 = 4 * T3 * velocityScale;
    double a23 = 5 * T4 * velocityScale;

    double a31 = 6 * T * accelScale;
    double a32 = 12 * T2 * accelScale;
    double a33 = 20 * T3 * accelScale;

    // Build a 3x3 matrix for polynomial coefficients
    Eigen::Matrix3d A;
    A << a11, a12, a13,
         a21, a22, a23,
         a31, a32, a33;
    double detA = A.determinant();

    // If determinant is near zero, we cannot solve reliably
    if (fabs(detA) < 1e-6)
    {
        return;
    }

    // S1,S2,S3 hold the difference between current and desired states for X
    double S1x = (xf - x0 - v0x * T - 0.5 * a0x * T2) * positionScale;
    double S2x = (-v0x - a0x * T) * velocityScale;
    double S3x = -a0x * accelScale;

    // Solve X polynomial coefficients using Cramer's rule
    Eigen::Matrix3d Ax = A;
    Ax.col(0) << S1x, S2x, S3x;
    double c3x = Ax.determinant() / detA;

    Ax = A;
    Ax.col(1) << S1x, S2x, S3x;
    double c4x = Ax.determinant() / detA;

    Ax = A;
    Ax.col(2) << S1x, S2x, S3x;
    double c5x = Ax.determinant() / detA;

    // Repeat for Y
    double S1y = (yf - y0 - v0y * T - 0.5 * a0y * T2) * positionScale;
    double S2y = (-v0y - a0y * T) * velocityScale;
    double S3y = -a0y * accelScale;

    Eigen::Matrix3d Ay = A;
    Ay.col(0) << S1y, S2y, S3y;
    double c3y = Ay.determinant() / detA;

    Ay = A;
    Ay.col(1) << S1y, S2y, S3y;
    double c4y = Ay.determinant() / detA;

    Ay = A;
    Ay.col(2) << S1y, S2y, S3y;
    double c5y = Ay.determinant() / detA;

    // Variables to remember old jerk values for smoothing between control steps
    static double prevJerkX = 0.0;
    static double prevJerkY = 0.0;

    // Compute raw jerk using the polynomial coefficients at the current elapsed time
    double jerkX = 6 * c3x + 24 * c4x * elapsedTime + 60 * c5x * elapsedTime * elapsedTime;
    double jerkY = 6 * c3y + 24 * c4y * elapsedTime + 60 * c5y * elapsedTime * elapsedTime;

    // Combine multiple factors to get a final dynamic factor that influences jerk smoothing
    double dynamicControlFactor = normalizedError * 0.65 +
            ((1.0 - absVx / HIGH_VEL_THRESHOLD) + (1.0 - absVy / HIGH_VEL_THRESHOLD)) * 0.125 +
            ((1.0 - absAx / HIGH_ACC_THRESHOLD) + (1.0 - absAy / HIGH_ACC_THRESHOLD)) * 0.05;

    // Final controlSmoothing factor is used to blend between old jerk and new jerk
    double controlSmoothing = qMax(0.15, qMin(0.85, dynamicControlFactor));

    // In the initial phase, we can ramp up smoothing a bit more aggressively
    if (isInitialPhase)
    {
        controlSmoothing = qMin(1.0, controlSmoothing * (1.2 + elapsedTime * 0.25));
    }

    // Blend old jerk with newly calculated jerk according to the smoothing factor
    jerkX = prevJerkX + (jerkX - prevJerkX) * controlSmoothing;
    jerkY = prevJerkY + (jerkY - prevJerkY) * controlSmoothing;

    // Update the previous jerk values for the next iteration
    prevJerkX = jerkX;
    prevJerkY = jerkY;

    // Clip jerk so it does not exceed MAX_INPUT for the current Level
    jerkX = qBound(-MAX_INPUT, jerkX, MAX_INPUT);
    jerkY = qBound(-MAX_INPUT, jerkY, MAX_INPUT);

    // Additional scaling factor on overall movement, influenced by error and velocities
    double movementScaling = qMax(0.35, qMin(1.15, dynamicControlFactor));

    // In the initial phase, allow a scaled ramp-up to get the pointer moving
    if (isInitialPhase)
    {
        movementScaling *= (elapsedTime / 1.5 * 1.6);
    }

    // Compute the final position for the mouse, centering in the middle of the screen,
    // then applying the jerk scaling. The pointer location is normalized by 'coeff[Level]'.
    double mouseX = xmax / 2 + (jerkX * (xmax / 2)) / coeff[Level] * movementScaling;
    double mouseY = ymax / 2 + (jerkY * (ymax / 2)) / coeff[Level] * movementScaling;

    // Ensure mouse coordinates stay within screen boundaries
    mouseX = qBound(0.0, mouseX, static_cast<double>(xmax));
    mouseY = qBound(0.0, mouseY, static_cast<double>(ymax));

    // Invoke system/Qt function to move the cursor
    moveMouseAbsolute(mouseX, mouseY);

    // Update live chart data for debugging or performance analysis
    updateChartData(elapsedTime, errorX, errorY, v0x, v0y, j0x, j0y);

    // Logging output to the UI, listing key variables for analysis
    QString msg = QString("[%1] Level %2 - Error: %3, T: %4\n"
                          "    VelX: %5, VelY: %6, AccX: %7, AccY: %8, JerkX: %9, JerkY: %10\n"
                          "    Pos Scale: %11, Vel Scale: %12, Acc Scale: %13\n"
                          "    Precision: %14, Smoothing: %15")
                      .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
                      .arg(Level)
                      .arg(currentError, 0, 'f', 2)
                      .arg(T, 0, 'f', 2)
                      .arg(v0x, 0, 'f', 2)
                      .arg(v0y, 0, 'f', 2)
                      .arg(a0x, 0, 'f', 2)
                      .arg(a0y, 0, 'f', 2)
                      .arg(jerkX, 0, 'f', 2)
                      .arg(jerkY, 0, 'f', 2)
                      .arg(positionScale, 0, 'f', 4)
                      .arg(velocityScale, 0, 'f', 4)
                      .arg(accelScale, 0, 'f', 4)
                      .arg(movementScaling, 0, 'f', 4)
                      .arg(controlSmoothing, 0, 'f', 4);
    ui->logText->append(msg);
}



void MainWindow::solveLevel4(const QJsonObject &data)
{
    // Time threshold that defines the 'initial phase' window
    // During this phase, movement and control logic can behave more aggressively
    const double INITIAL_PHASE_TIME = 1.4;

    // Elapsed time since this control logic started
    double elapsedTime = data["ttime"].toDouble();

    // Check whether we are still in the initial phase
    bool isInitialPhase = (elapsedTime < INITIAL_PHASE_TIME);

    // Current "Level" determines which entry in 'coeff' is used to scale pointer movement
    int Level = currentLevel;

    // Constants used in deciding how we shape the polynomial control
    const double FINAL_APPROACH_ERROR = 40.0;    // If error < 40, we consider it the final approach
    const double VERY_CLOSE_ERROR = 20.0;        // If error < 20, we are very close to the target
    const double HIGH_VEL_THRESHOLD = 150.0;     // Velocity threshold for "high velocity"
    const double HIGH_ACC_THRESHOLD = 500.0;     // Acceleration threshold for "high acceleration"
    const double HIGH_JERK_THRESHOLD = 1000.0;   // Jerk threshold for "high jerk"

    // Extract arrays from the incoming JSON that hold position, velocity, etc.
    QJsonArray vectorxArray = data["vectorx"].toArray();
    QJsonArray vectoryArray = data["vectory"].toArray();

    // 'coeff' array for different Levels, sets the maximum input scale
    double coeff[] = {1, 1000, 100, 10, 1, 1, 1, 1, 1, 1, 1};
    double MAX_INPUT = coeff[Level];

    // Current state extracted from the arrays: position (x0,y0), velocity (v0x,v0y),
    // acceleration (a0x,a0y), jerk (j0x,j0y), snap (s0x,s0y)
    double x0 = vectorxArray[0].toDouble();
    double y0 = vectoryArray[0].toDouble();
    double v0x = vectorxArray[1].toDouble();
    double v0y = vectoryArray[1].toDouble();
    double a0x = vectorxArray[2].toDouble();
    double a0y = vectoryArray[2].toDouble();
    double j0x = vectorxArray[3].toDouble();
    double j0y = vectoryArray[3].toDouble();
    double s0x = vectorxArray[4].toDouble();
    double s0y = vectoryArray[4].toDouble();

    // Desired final state (target position), with zero velocity, acceleration, jerk for finishing
    double xf = targetX;
    double yf = targetY;
    double vf = 0.0;
    double af = 0.0;
    double jf = 0.0;

    double errorX = xf - x0;
    double errorY = yf - y0;

    // Calculate the current tracking error, velocity magnitude, etc.
    double currentError = sqrt(pow(xf - x0, 2) + pow(yf - y0, 2));
    double velocityMag = sqrt(v0x * v0x + v0y * v0y);
    double accMagnitude = sqrt(a0x * a0x + a0y * a0y);
    double jerkMagnitude = sqrt(j0x * j0x + j0y * j0y);

    // Keep track of the maximum error over time for normalization
    static double maxError = 0.0;
    if (currentError > maxError)
    {
        maxError = currentError;
    }
    else if (isInitialPhase) 
    {
        // During the initial phase, slightly inflate the maxError to keep scaling from jumping
        maxError = currentError * 1.5;
    }
    double normalizedError = maxError > 0 ? currentError / maxError : 1.0;

    // Adaptive time horizon T for polynomial planning. We keep it in [MIN_T, MAX_T].
    static double T = 30.0;        // Default initial guess
    const double MIN_T = 12.0;
    const double MAX_T = 35.0;
    const double ERROR_THRESHOLD = 90.0;

    if (!isInitialPhase)
    {
        // We only update T after leaving the initial phase. This helps reduce abrupt changes early on.

        // Calculate direction to the target and the distance
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = sqrt(dx * dx + dy * dy);

        // Unit direction toward the target
        double dirX = dx / distance;
        double dirY = dy / distance;

        // Project velocity and acceleration along that direction
        double vel = v0x * dirX + v0y * dirY;
        double acc = a0x * dirX + a0y * dirY;

        // Estimate how long to brake to zero velocity if they oppose (acc != 0 and vel * acc < 0)
        double timeToZeroVelocity = (acc != 0 && vel * acc < 0) ? -vel / acc : 0;

        // Compute leftover distance after a hypothetical braking phase
        double distanceAfterBraking = distance - (vel * timeToZeroVelocity +
                                                 0.5 * acc * timeToZeroVelocity * timeToZeroVelocity);

        // Estimate time to travel the leftover distance at roughly half the current velocity
        double timeToTraverseRemaining = distanceAfterBraking / (0.5 * vel);

        // Combine times to get a naive estimate of total time to reach the target
        double estimatedTimeToTarget = timeToZeroVelocity + timeToTraverseRemaining;

        // Introduce scaling factors based on velocity, error, and acceleration to refine T
        double velocityFactor = qMax(0.0, 1.0 - velocityMag / HIGH_VEL_THRESHOLD);
        double errorFactor = qMax(0.0, 1.0 - currentError / ERROR_THRESHOLD);
        double accFactor = qMax(0.0, 1.0 - accMagnitude / HIGH_ACC_THRESHOLD);

        // Weighted combination for final time estimate
        estimatedTimeToTarget = estimatedTimeToTarget * (0.7 +
                                                         velocityFactor * 0.3 +
                                                         errorFactor * 0.2 +
                                                         accFactor * 0.1);

        // Clamp the new estimate within valid bounds
        estimatedTimeToTarget = qBound(MIN_T, estimatedTimeToTarget, MAX_T);

        // Apply a smoothing factor so T changes gradually
        double alpha = 0.12;


        if (currentError < VERY_CLOSE_ERROR)
        {
            // speed up the change if we’re very close so the horizon does not increase while we are within target
            alpha *= 1.5;   
        }


        // Exponential smoothing for T
        T = T * (1 - alpha) + estimatedTimeToTarget * alpha;
    }

    // Compute powers of T for the polynomial approach up to T^7
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    double T6 = T5 * T;
    double T7 = T6 * T;

    // Additional ratio used to scale polynomial terms based on how T compares to its max
    double tRatio = T / MAX_T;
    // Position, velocity, acceleration, jerk scales are all adjusted by the ratio
    double positionScale = 1.0 / (T4 * tRatio);
    double velocityScale = 1.0 / (T3 * sqrt(tRatio));
    double accelScale = 1.0 / (T2 * tRatio);
    double jerkScale = 1.0 / (T * tRatio);

    // Coefficients for the 4x4 matrix A used in the polynomial solver
    // These reflect final position, velocity, acceleration, jerk constraints
    double a11 = T4 * positionScale;
    double a12 = T5 * positionScale;
    double a13 = T6 * positionScale;
    double a14 = T7 * positionScale;

    double a21 = 4 * T3 * velocityScale;
    double a22 = 5 * T4 * velocityScale;
    double a23 = 6 * T5 * velocityScale;
    double a24 = 7 * T6 * velocityScale;

    double a31 = 12 * T2 * accelScale;
    double a32 = 20 * T3 * accelScale;
    double a33 = 30 * T4 * accelScale;
    double a34 = 42 * T5 * accelScale;

    double a41 = 24 * T * jerkScale;
    double a42 = 60 * T2 * jerkScale;
    double a43 = 120 * T3 * jerkScale;
    double a44 = 210 * T4 * jerkScale;

    // Build the 4x4 matrix
    Eigen::Matrix4d A;
    A << a11, a12, a13, a14,
         a21, a22, a23, a24,
         a31, a32, a33, a34,
         a41, a42, a43, a44;
    double detA = A.determinant();

    // If determinant is near zero, the system is ill-conditioned and we should not solve
    if (fabs(detA) < 1e-6)
    {
        return;
    }

    // Additional factors to adapt the final scale based on velocity, acceleration, jerk
    double velocityScaleFactor = qMax(0.5, qMin(1.1, tRatio));
    double accFactor = qMax(0.8, qMin(1.2, 1.0 - accMagnitude / HIGH_ACC_THRESHOLD));
    double jerkFactor = qMax(0.8, qMin(1.2, 1.0 - jerkMagnitude / HIGH_JERK_THRESHOLD));

    // Combine everything into an overall adaptiveScale
    double adaptiveScale = qMin(1.0 + currentError * 0.006 * (1 / tRatio), 1.6) *
                           velocityScaleFactor * accFactor * jerkFactor;

    // If still in the initial phase, further inflate the scaling to quickly get moving
    if (isInitialPhase) 
    {
        adaptiveScale *= (1.0 + (1.0 - elapsedTime / INITIAL_PHASE_TIME) * 0.3);
    }

    // Define the S terms for X using the polynomial formula with an extra adaptiveScale
    double S1x = (xf - x0 - v0x * T - 0.5 * a0x * T2 - j0x * T3 / 6.0) * adaptiveScale * positionScale;
    double S2x = (-v0x - a0x * T - 0.5 * j0x * T2) * adaptiveScale * velocityScale;
    double S3x = (-a0x - j0x * T) * adaptiveScale * accelScale;
    double S4x = -j0x * adaptiveScale * jerkScale;

    // Solve for the polynomial coefficients c4x..c7x using determinant swaps (Cramer's rule)
    Eigen::Vector4d Sx(S1x, S2x, S3x, S4x);
    Eigen::Matrix4d tempA = A;

    tempA.col(0).swap(Sx);
    double detC4x = tempA.determinant();

    tempA = A;
    tempA.col(1).swap(Sx);
    double detC5x = tempA.determinant();

    tempA = A;
    tempA.col(2).swap(Sx);
    double detC6x = tempA.determinant();

    tempA = A;
    tempA.col(3).swap(Sx);
    double detC7x = tempA.determinant();

    double c4x = detC4x / detA;
    double c5x = detC5x / detA;
    double c6x = detC6x / detA;
    double c7x = detC7x / detA;

    // Repeat for Y
    double S1y = (yf - y0 - v0y * T - 0.5 * a0y * T2 - j0y * T3 / 6.0) * adaptiveScale * positionScale;
    double S2y = (-v0y - a0y * T - 0.5 * j0y * T2) * adaptiveScale * velocityScale;
    double S3y = (-a0y - j0y * T) * adaptiveScale * accelScale;
    double S4y = -j0y * adaptiveScale * jerkScale;

    Eigen::Vector4d Sy(S1y, S2y, S3y, S4y);

    tempA = A;
    tempA.col(0).swap(Sy);
    double detC4y = tempA.determinant();

    tempA = A;
    tempA.col(1).swap(Sy);
    double detC5y = tempA.determinant();

    tempA = A;
    tempA.col(2).swap(Sy);
    double detC6y = tempA.determinant();

    tempA = A;
    tempA.col(3).swap(Sy);
    double detC7y = tempA.determinant();

    double c4y = detC4y / detA;
    double c5y = detC5y / detA;
    double c6y = detC6y / detA;
    double c7y = detC7y / detA;

    // Memory of the previous snap (4th derivative) to smooth transitions
    static double prevSnapX = 0.0;
    static double prevSnapY = 0.0;

    // Calculate the desired new snap from polynomial at current elapsed time
    // Snap is derivative of jerk (the 4th derivative of position).
    double sdesx = 24 * c4x + 120 * c5x * elapsedTime + 360 * c6x * elapsedTime * elapsedTime + 
                   840 * c7x * elapsedTime * elapsedTime * elapsedTime;
    double sdesy = 24 * c4y + 120 * c5y * elapsedTime + 360 * c6y * elapsedTime * elapsedTime + 
                   840 * c7y * elapsedTime * elapsedTime * elapsedTime;

    // Weighted factor to blend old snap with newly calculated snap
    double smoothingFactor = qMax(0.1, qMin(0.8,
                                            normalizedError * 0.6 +
                                                (1.0 - velocityMag / HIGH_VEL_THRESHOLD) * 0.2 +
                                                (1.0 - accMagnitude / HIGH_ACC_THRESHOLD) * 0.1 +
                                                (1.0 - jerkMagnitude / HIGH_JERK_THRESHOLD) * 0.1));

    // If we are in the initial phase, allow more aggressive ramp-up
    if (isInitialPhase)
    {
        smoothingFactor = qMin(1.0, smoothingFactor * (1.2 + elapsedTime * 0.2));
    }
    // As we approach the target, slightly increase smoothing to ease into final position
    else if (currentError < FINAL_APPROACH_ERROR)
    {
        double approachProgress = 1.0 - (currentError - VERY_CLOSE_ERROR) /
                                          (FINAL_APPROACH_ERROR - VERY_CLOSE_ERROR);
        smoothingFactor = qMin(1.0, smoothingFactor * (1.2 + approachProgress * 0.3));
    }

    // Blend the current snap with the previously stored snap
    sdesx = prevSnapX + (sdesx - prevSnapX) * smoothingFactor;
    sdesy = prevSnapY + (sdesy - prevSnapY) * smoothingFactor;

    // Update old snap for next iteration
    prevSnapX = sdesx;
    prevSnapY = sdesy;

    // This is the final polynomial-based control input for X and Y
    double controlInputX = sdesx;
    double controlInputY = sdesy;

    // Scale the maximum possible control input based on distance to the target
    double dynamicMaxInput = MAX_INPUT * qMin(1.0 + currentError * 0.012, 1.8);
    if (isInitialPhase)
    {
        // In the initial phase, further inflate to get faster movement from the start
        dynamicMaxInput *= (1.0 + (1.0 - elapsedTime / INITIAL_PHASE_TIME) * 0.4);
    }
    else if (currentError < FINAL_APPROACH_ERROR)
    {
        // As we get closer, we allow some extra push but also might refine further if VERY close
        double approachFactor = 1.0 - currentError / FINAL_APPROACH_ERROR;
        dynamicMaxInput *= (1.1 + approachFactor * 0.2);

        if (currentError < VERY_CLOSE_ERROR)
        {
            // If we're extremely close, add one more layer of potential scaling
            double fineFactor = 1.0 - currentError / VERY_CLOSE_ERROR;
            dynamicMaxInput *= (1.3 + fineFactor * 0.2);
        }
    }

    // Ensure control remains within +/- dynamicMaxInput
    controlInputX = qBound(-dynamicMaxInput, controlInputX, dynamicMaxInput);
    controlInputY = qBound(-dynamicMaxInput, controlInputY, dynamicMaxInput);

    // The final precision factor for movement, influenced by error, velocity, acceleration, jerk
    double precisionFactor = qMax(0.3, qMin(1.1,
                                            normalizedError * 0.6 +
                                                (1.0 - velocityMag / HIGH_VEL_THRESHOLD) * 0.2 +
                                                (1.0 - accMagnitude / HIGH_ACC_THRESHOLD) * 0.1 +
                                                (1.0 - jerkMagnitude / HIGH_JERK_THRESHOLD) * 0.1));

    // In the initial phase, ramp up quickly
    if (isInitialPhase)
    {
        precisionFactor *= (elapsedTime / INITIAL_PHASE_TIME * 1.5);
    }
    // As we get closer to the target, we can also adapt the precision upward
    else if (currentError < FINAL_APPROACH_ERROR)
    {
        double progress = 1.0 - currentError / FINAL_APPROACH_ERROR;
        precisionFactor *= (1.1 + progress * 0.3);

        if (currentError < VERY_CLOSE_ERROR)
        {
            double fineProgress = 1.0 - currentError / VERY_CLOSE_ERROR;
            precisionFactor *= (1.3 + fineProgress * 0.2);
        }
    }

    // Compute the final position for the mouse pointer
    // We scale it around the screen center, then apply precisionFactor and the coefficient
    double mouseX = xmax / 2 + (controlInputX * (xmax / 2)) / coeff[Level] * precisionFactor;
    double mouseY = ymax / 2 + (controlInputY * (ymax / 2)) / coeff[Level] * precisionFactor;

    // Ensure the pointer does not go off screen
    mouseX = qBound(0.0, mouseX, static_cast<double>(xmax));
    mouseY = qBound(0.0, mouseY, static_cast<double>(ymax));

    // Execute the actual pointer move
    moveMouseAbsolute(mouseX, mouseY);

    updateChartData(elapsedTime, errorX, errorY, v0x, v0y, s0x, s0y);

    QString msg = QString("[%1] Level %2 - Error: %3, T: %4, Scale: %5\n"
                          "    Vel: %6, Acc: %7, Jerk: %8, Snap: %9\n"
                          "    Precision: %10, Smoothing: %11")
                      .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
                      .arg(Level)
                      .arg(currentError, 0, 'f', 2)
                      .arg(T, 0, 'f', 2)
                      .arg(adaptiveScale, 0, 'f', 2)
                      .arg(velocityMag, 0, 'f', 2)
                      .arg(accMagnitude, 0, 'f', 2)
                      .arg(jerkMagnitude, 0, 'f', 2)
                      .arg(sqrt(sdesx * sdesx + sdesy * sdesy), 0, 'f', 2)
                      .arg(precisionFactor, 0, 'f', 2)
                      .arg(smoothingFactor, 0, 'f', 2);
    ui->logText->append(msg);
}
