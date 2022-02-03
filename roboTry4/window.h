#ifndef WINDOW
#define WINDOW

#define _USE_MATH_DEFINES

#include <functional>
#include <QWidget>
#include <QMediaPlayer>

#include "graphics.h"

class QPlainTextEdit;
class QLineEdit;

class Window;

class Widget : public QWidget
{
    Q_OBJECT

    mssm::Graphics *_graphics;
public:
    Window   *_parent;

public:
    Widget(mssm::Graphics *graphics, Window *parent);

public slots:
    void animate();

protected:
    void wheelEvent(QWheelEvent *event) override;
    void paintEvent(QPaintEvent * event) override;
    void mousePressEvent(QMouseEvent * event) override ;
    void mouseReleaseEvent(QMouseEvent * event) override ;
    void mouseMoveEvent(QMouseEvent * event) override ;
    void keyPressEvent(QKeyEvent * event) override;
    void keyReleaseEvent(QKeyEvent * event) override;
private:
    std::chrono::milliseconds::rep lastTime;
};

class Window : public QWidget
{
    Q_OBJECT

    std::mutex wlock;
    std::condition_variable cv;

    Widget         *graphicsWidget;
    QPlainTextEdit *streamOutBox;
    QLineEdit      *streamInBox;
    mssm::Graphics *graphics;

    bool            hasEnteredText{false};
    std::string     enteredText;
    bool            usingStreamIO{false};

public:
    Window(mssm::Graphics *graphics, std::string title);

    void appendOutputText(const std::string& txt);
    std::string getInputText();

public slots:
    void textEntered();
    void musicStateChanged(QMediaPlayer::State state);
};

class Worker : public QObject {
    Q_OBJECT

    mssm::Graphics *graphics;
    Window*         window;
    QThread*        thread;
public:
    Worker(QThread* t, mssm::Graphics* g, Window *w);
   ~Worker();

public slots:
    void process();

signals:
    void finished();
    void error(QString err);

private:
    // add your variables here
};

#endif // WINDOW

