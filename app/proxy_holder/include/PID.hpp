#pragma once

#include <functional>

template <class T>
class PIDController
{
public:
    PIDController(double p=1, double i=1, double d=1);
    T tick(T currentFeedback, T target);
    T getOutput();
    T getFeedback();
    T getError();
    void setEnabled(bool e);
    bool isEnabled();
    T getProportionalComponent();
    T getIntegralComponent();
    T getDerivativeComponent();
    void setMaxIntegralCumulation(T max);
    T getMaxIntegralCumulation();
    T getIntegralCumulation();

    void setInputBounded(bool bounded);
    bool isInputBounded();
    void setInputBounds(T lower, T upper);
    T getInputLowerBound();
    T getInputUpperBound();
    void setOutputBounded(bool bounded);
    bool isOutputBounded();
    void setOutputBounds(T lower, T upper);
    T getOutputLowerBound();
    T getOutputUpperBound();
    void setFeedbackWrapped(bool wrap);
    bool isFeedbackWrapped();
    void setFeedbackWrapBounds(T lower, T upper);
    T getFeedbackWrapLowerBound();
    T getFeedbackWrapUpperBound();

    void setPID(double p, double i, double d);
    void setP(double p);
    void setI(double i);
    void setD(double d);
    double getP();
    double getI();
    double getD();
    void registerTimeFunction(unsigned long (*getSystemTime)());

private:
    double _p;
    double _i;
    double _d;
    T output;
    bool enabled;
    T currentFeedback;
    T lastFeedback;
    T error;
    T lastError;
    long currentTime;
    long lastTime;
    T integralCumulation;
    T maxCumulation;
    T cycleDerivative;

    bool inputBounded;
    T inputLowerBound;
    T inputUpperBound;
    bool outputBounded;
    T outputLowerBound;
    T outputUpperBound;
    bool feedbackWrapped;
    T feedbackWrapLowerBound;
    T feedbackWrapUpperBound;

    bool timeFunctionRegistered;
    unsigned long (*_getSystemTime)();
};

// #include "PID.cpp"