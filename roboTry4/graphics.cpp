#define _USE_MATH_DEFINES

#include "graphics.h"
#include "window.h"

#include <iostream>

#include <functional>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <random>
#include <cmath>

#include <QGridLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QApplication>
#include <QSurfaceFormat>
#include <QWidget>
#include <QTimer>
#include <QPainter>
#include <QPointF>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QPixmap>
#include <QDir>
#include <QMediaPlayer>
#include <QSoundEffect>
#include <QThread>
#include <QBitmap>

#include <vector>
#include <iterator>
#include <utility>

using namespace mssm;
using namespace std;



template <typename T>
typename std::vector<T>::iterator append(const std::vector<T>& src, std::vector<T>& dest)
{
    typename std::vector<T>::iterator result;

    if (dest.empty()) {
        dest = src;
        result = std::begin(dest);
    } else {
        result = dest.insert(std::end(dest), cbegin(src), cend(src));
    }

    return result;
}

template <typename T>
typename std::vector<T>::iterator append(std::vector<T>&& src, std::vector<T>& dest)
{
    typename std::vector<T>::iterator result;

    if (dest.empty()) {
        dest = std::move(src);
        result = std::begin(dest);
    } else {
        result = dest.insert(std::end(dest),
                             std::make_move_iterator(std::begin(src)),
                             std::make_move_iterator(std::end(src)));
    }

    src.clear();
    src.shrink_to_fit();

    return result;
}

#include <streambuf>

// todo:  http://stackoverflow.com/questions/5642063/inheriting-ostream-and-streambuf-problem-with-xsputn-and-overflow

class iStreamBuf : public std::streambuf
{
public:
    explicit iStreamBuf(std::function<std::string ()> reader, std::size_t buff_sz = 256, std::size_t put_back = 8);

    iStreamBuf(const iStreamBuf &) = delete;
    iStreamBuf &operator= (const iStreamBuf &) = delete;

private:
    // overrides base class underflow()
    int_type underflow();

    size_t readMore(char *dst, size_t toRead);

private:
    std::function<std::string ()> _reader;
    const std::size_t _put_back;
    std::vector<char> _buffer;
    std::string _source;
};

using std::size_t;

iStreamBuf::iStreamBuf(std::function<std::string ()> reader,
                       size_t buff_sz, size_t put_back) :
    _reader{reader},
    _put_back(std::max(put_back, size_t(1))),
    _buffer(std::max(buff_sz, _put_back) + _put_back)
{
    auto end = &_buffer.front() + _buffer.size();
    setg(end, end, end);
}

size_t iStreamBuf::readMore(char *dst, size_t toRead)
{
    if (_source.empty())
    {
        _source = _reader();
    }

    size_t count = std::min(toRead, _source.size());

    if (count == 0)
    {
        return 0;
    }

    std::copy(_source.begin(), _source.begin()+count, dst);

    _source.erase(_source.begin(), _source.begin()+count);

    return count;
}

std::streambuf::int_type iStreamBuf::underflow()
{
    if (gptr() < egptr())
    {
        return traits_type::to_int_type(*gptr());
    }

    char *base = &_buffer.front();
    char *start = base;

    if (eback() == base) // true when this isn't the first fill
    {
        // Make arrangements for putback characters
        std::copy(egptr()-_put_back, egptr(), base);
        //        std::memmove(base, egptr() - put_back_, put_back_);
        start += _put_back;
    }

    // start is now the start of the buffer, proper.
    // Read from fptr_ in to the provided buffer
    size_t n = readMore(start, _buffer.size() - (start - base));
    if (n == 0)
    {
        return traits_type::eof();
    }

    // Set buffer pointers
    setg(base, start, start + n);

    return traits_type::to_int_type(*gptr());
}


class oStreamBuf : public std::streambuf
{
public:
    explicit oStreamBuf(std::function<bool (const std::string& str)> writer, std::size_t buff_sz = 256);

    oStreamBuf(const iStreamBuf &) = delete;
    oStreamBuf &operator= (const iStreamBuf &) = delete;

protected:
    bool reallyWrite();

private:
    int_type overflow(int_type ch);
    int sync();

private:
    std::function<bool (const std::string& str)> _writer;
    std::vector<char> _buffer;
};

oStreamBuf::oStreamBuf(std::function<bool (const std::string& str)> writer, std::size_t buff_sz) :
    _writer(writer),
    _buffer(buff_sz + 1)
{
    char *base = &_buffer.front();
    setp(base, base + _buffer.size() - 1); // -1 to make overflow() easier
}

oStreamBuf::int_type oStreamBuf::overflow(int_type ch)
{
    if (ch != traits_type::eof())
    {
        //        std::assert(std::less_equal<char *>()(pptr(), epptr()));
        *pptr() = ch;
        pbump(1);
        if (reallyWrite())
        {
            return ch;
        }
    }

    return traits_type::eof();
}

int oStreamBuf::sync()
{
    return reallyWrite() ? 0 : -1;
}

bool oStreamBuf::reallyWrite()
{
    std::ptrdiff_t n = pptr() - pbase();

    pbump(-n);

    string str(pbase(), n);

    return _writer(str);
}

namespace mssm
{

Sound::Sound(const string &filename)
    : sound{std::make_shared<SoundInternal>(mssm::findFile(filename))}
{
}

Image::Image(int width, int height, Color c)
{
    set(width, height, c);
}

Image::Image(const std::string& filename)
{
    auto fpath = mssm::findFile(filename);
    if (!fpath.empty()) {
        pixmap = std::make_shared<QPixmap>(QString::fromStdString(fpath));
    }
    else
    {
        set(20,20,RED);
    }
}

void Image::load(const std::string& fileName)
{
    *this = Image{fileName};
}

void Image::save(const string &pngFileName)
{
    QFile file(pngFileName.c_str());
    file.open(QIODevice::WriteOnly);
    pixmap->save(&file, "PNG");
}

void Image::set(const std::vector<Color>& pixels, int width, int height)
{
    QImage img((uchar*)pixels.data(), width, height, QImage::Format_RGBA8888);

    pixmap = std::make_shared<QPixmap>();

    *pixmap = QPixmap::fromImage(img);
}

void Image::set(int width, int height, Color c)
{
    std::vector<Color> pixels(width*height, c);
    set(pixels, width, height);
}

int Image::width()
{
    return pixmap ? pixmap->width() : 0;
}

int Image::height()
{
    return pixmap ? pixmap->height() : 0;
}

SoundInternal::SoundInternal(const std::string& _filename) : filename(_filename)
{
    //        sound = std::make_shared<QSoundEffect>(parent);
}


void SoundInternal::release()
{
    sound.reset();
}

bool SoundInternal::play(QObject* parent)
{
    bool created = false;

    if (!sound)
    {
        sound = std::make_shared<QSoundEffect>(parent);
        created = true;
    }

    switch (sound->status())
    {
    case QSoundEffect::Status::Null:
        sound->setSource(QUrl::fromLocalFile(QString::fromStdString(filename)));
        sound->setLoopCount(1);
        sound->play();
        break;
    case QSoundEffect::Status::Ready:
        sound->setLoopCount(1);
        sound->play();
        break;
    case QSoundEffect::Status::Loading:
        // cout << "Still Loading Sound!" << endl;
        break;
    case QSoundEffect::Status::Error:
        cout << "Error playing sound: " << filename << endl;
        break;
    }

    return created;
}

int SoundInternal::status()
{
    if (!sound)
    {
        return 0;
    }
    return (int)sound->status();
}

void helper(Graphics* g, Window *window, QObject* thread)
{
    g->callMainFunc(window, thread);
}

class Grob
{
public:
    virtual ~Grob() {}
    virtual void draw(QPainter *painter) = 0;
};

class GrobPoint : public Grob
{
public:
    Vec2d p;
    Color c;
public:
    GrobPoint(Vec2d _p, Color _c) : p(_p), c(_c) {}
    virtual void draw(QPainter *painter);
};

class GrobPoints : public Grob
{
public:
    std::vector<QPointF> points;
    Color c;
public:
    GrobPoints(const std::vector<Vec2d>& points, Color c);
    virtual void draw(QPainter *painter);
};


class GrobLine : public Grob
{
    double x1;
    double y1;
    double x2;
    double y2;
    Color  c;
public:
    GrobLine(double _x1, double _y1, double _x2, double _y2, Color _c)
        : x1(_x1), y1(_y1), x2(_x2), y2(_y2), c(_c) {}
    virtual void draw(QPainter *painter);
};

class GrobEllipse : public Grob
{
    double   x;
    double   y;
    double   w;
    double   h;
    Color    c;
    Color    fill;
    double   a;
    double   alen;
    int      type; // 0 = ellipse, 1 = arc, 2 = chord, 3 = pie
public:
    GrobEllipse(double _x, double _y, double _w, double _h, Color _c, Color _fill = TRANSPARENT)
        : x(_x), y(_y), w(_w), h(_h), c(_c), fill(_fill), a{0}, alen{M_PI*2}, type{0} {}
    GrobEllipse(double _x, double _y, double _w, double _h, double _a, double _alen, int _type, Color _c, Color _fill = TRANSPARENT)
        : x(_x), y(_y), w(_w), h(_h), c(_c), fill(_fill), a{_a}, alen{_alen}, type{_type}  {}
    virtual void draw(QPainter *painter);
};

class GrobPoly : public Grob
{
    std::vector<QPointF> points;
    Color    c;
    Color    fill;
    bool     closed;
public:
    GrobPoly(const std::vector<Vec2d>& points, Color outlineColor, bool closed, Color fillColor = TRANSPARENT);
    virtual void draw(QPainter *painter);
};

class GrobRect : public Grob
{
    QRectF   rect;
    Color    c;
    Color    fill;
public:
    GrobRect(Vec2d upperLeft, double w, double h, Color outlineColor, Color fillColor = TRANSPARENT)
        : rect{upperLeft.x, -upperLeft.y-h, w, h}, c{outlineColor}, fill{fillColor} {}
    virtual void draw(QPainter *painter);
};

class GrobImage : public Grob
{
public:
    double  x;
    double  y;
    std::shared_ptr<QPixmap> image;
    int     srcx = -1;
    int     srcy = -1;
    int     srcw = -1;
    int     srch = -1;
    double  dstw = 0;
    double  dsth = 0;
public:
    GrobImage(double _x, double _y, std::shared_ptr<QPixmap> _image)
        : x(_x), y(_y), image(_image) {}
    void draw(QPainter *painter);
};

class GrobText : public Grob
{
    double x;
    double y;
    double sz;
    std::string text;
    Color  color;
public:
    GrobText(double x, double y, double sz, const std::string& text, Color textColor);
    virtual void draw(QPainter *painter);
};




}

Worker::Worker(QThread *t, mssm::Graphics* g, Window *w) : graphics(g), window(w), thread(t)
{
    // you could copy data from constructor arguments to internal variables here.
}

// --- DECONSTRUCTOR ---
Worker::~Worker() {
    // free resources

    //cout << "Worker Destructor Called" << endl;
}

// --- PROCESS ---
// Start processing data.
void Worker::process()
{
    // allocate resources using new here
    helper(graphics, window, this);

    //cout << "Emitting FInished" << endl;
    emit finished();

    //cout << "After Emitting FInished" << endl;

    thread->quit();
}

Graphics::Graphics(std::string title, int width, int height, std::function<void (Graphics&)> mainThreadFunc)
    :
    Graphics(title, width, height, [mainThreadFunc](Graphics& g, QObject*) { mainThreadFunc(g); })
{
}


Graphics::Graphics(std::string title, int width, int height,
                   std::function<void (Graphics&, QObject*)> mainFunc,
                   std::function<void (Graphics&, QObject*, int)> uiFunc)
    : mersenneTwister(randDevice()),
      in{new iStreamBuf([this]() { return getInputText(); })},
      out{new oStreamBuf([this](const std::string& txt) { return appendOutputText(txt); })}
{   
    qRegisterMetaType<std::string>();

    mersenneTwister.seed((std::chrono::system_clock::now().time_since_epoch()).count()); // should'nt be necesary... bug in GCC?

    this->uiFunc   = uiFunc;
    this->mainFunc = mainFunc;

    _width = width;
    _height = height;

    this->title = title;
    background = BLACK;

    start_time =
            std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();

    int argc = 0;

    QApplication app(argc, nullptr);

    QSurfaceFormat fmt;
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    isPressed.resize(128);
    wasPressed.resize(128);

    Window window(this, getTitle());

    window.show();

    QThread* thread = new QThread;
    Worker*  worker = new Worker(thread, this, &window);

    worker->moveToThread(thread);

    QObject::connect(thread, SIGNAL(started()), worker, SLOT(process()));
    QObject::connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
    QObject::connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    QObject::connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    //cout << "Starting Thread" << endl;

    thread->start();

    app.exec();

    // clean up sound cache

    for (Sound s : soundCache)
    {
        s.sound->release();
    }

    if (musicPlayer)
    {
        musicPlayer->stop();
        musicPlayer.reset();
    }

    setClosed();

    thread->wait();

    pendingPlugins.clear();
    activePlugins.clear();
}

Graphics::~Graphics()
{
}


std::string mssm::findFile(const std::string& filename)
{
    auto qfilename = QString::fromStdString(filename);

    QFileInfo check_file(qfilename);

    if (check_file.exists() && check_file.isFile())
    {
        return QDir(qfilename).absolutePath().toStdString();
    }

    QDir dir;

    dir.absolutePath();

    if (dir.exists(qfilename)) {
        return dir.absoluteFilePath(qfilename).toStdString();
    }

    dir.cd("data");

    if (dir.exists(qfilename)) {
        return dir.absoluteFilePath(qfilename).toStdString();
    }

    dir.cdUp();
    dir.cd("assets");

    if (dir.exists(qfilename)) {
        return dir.absoluteFilePath(qfilename).toStdString();
    }

    return "";
}

std::string Graphics::currentPath(const std::string& fileName)
{
    QDir dir;

    if (!fileName.empty())
    {
        return dir.absoluteFilePath(QString::fromStdString(fileName)).toStdString();
    }
    return dir.absolutePath().toStdString();
}

std::string Graphics::programName()
{
    return QFileInfo( QCoreApplication::applicationFilePath() ).fileName().toStdString();
}

bool Graphics::isKeyPressed(Key k)
{
    return isKeyPressed((int)k);
}

int Graphics::randomInt(int minVal, int maxVal)
{
    std::uniform_int_distribution<int> dist(minVal, maxVal);
    return dist(mersenneTwister);
}

double Graphics::randomDouble(double minVal, double maxVal)
{
    std::uniform_real_distribution<double> dist(minVal, maxVal);
    return dist(mersenneTwister);
}

bool Graphics::randomTrue(double pct)
{
    if (pct <= 0.0)
    {
        return false;
    }
    if (pct >= 1.0)
    {
        return true;
    }
    std::uniform_real_distribution<double> dist(0, 1);
    return dist(mersenneTwister) <= pct;
}

std::ostream& mssm::operator<<(std::ostream& os, const Event& evt)
{
    os << "Evt: ";

    switch (evt.evtType) {
    case EvtType::KeyPress: os << "KeyPress"; break;
    case EvtType::KeyRelease: os << "KeyRelease"; break;
    case EvtType::PluginCreated: os << "PluginCreated"; break;
    case EvtType::PluginClosed: os << "PluginClosed"; break;
    case EvtType::PluginMessage: os << "PluginMessage"; break;
    case EvtType::MouseMove: os << "MouseMove"; break;
    case EvtType::MousePress: os << "MousePress"; break;
    case EvtType::MouseRelease: os << "MouseRelease"; break;
    case EvtType::MouseWheel: os << "MouseWheel"; break;
    }



    os << " x: " << evt.x << " y: " << evt.y << " arg: " << evt.arg;

    if (static_cast<int>(evt.mods) & static_cast<int>(ModKey::Ctrl))  { os << " <CTRL>";  }
    if (static_cast<int>(evt.mods) & static_cast<int>(ModKey::Alt))   { os << " <ALT>";   }
    if (static_cast<int>(evt.mods) & static_cast<int>(ModKey::Shift)) { os << " <SHIFT>"; }

    os << " data: " << evt.data << " pluginId: " << evt.pluginId;

    return os;
}

void Graphics::handleEvent(int x, int y, EvtType evtType, ModKey mods, int arg, int pluginId, const std::string& data)
{
    std::unique_lock<std::mutex> lock(glock);

    postEvent(x, _height-y, evtType, mods, arg, pluginId, data);
}

void Graphics::postEvent(int x, int y, EvtType evtType, ModKey mods, int arg, int pluginId, const std::string& data)
{
    Event evt{evtType,x,y,mods,arg, pluginId, data};

    if (closed)
    {
        return;
    }

    switch (evtType)
    {
    case EvtType::KeyPress:
        if (arg < 128)
        {
            isPressed[arg]  = true;
        }
        else
        {
            switch (arg)
            {
            case Qt::Key_Left:  isPressed[(int)Key::Left]   = true; evt.arg = (int)Key::Left;   break;
            case Qt::Key_Right: isPressed[(int)Key::Right]  = true; evt.arg = (int)Key::Right;  break;
            case Qt::Key_Up:    isPressed[(int)Key::Up]     = true; evt.arg = (int)Key::Up;     break;
            case Qt::Key_Down:  isPressed[(int)Key::Down]   = true; evt.arg = (int)Key::Down;   break;
            case Qt::Key_Shift: isPressed[(int)Key::Shift]  = true; evt.arg = (int)Key::Shift;  break;
            case Qt::Key_Alt:   isPressed[(int)Key::Alt]    = true; evt.arg = (int)Key::Alt;    break;
            case Qt::Key_Control: isPressed[(int)Key::Ctrl] = true; evt.arg = (int)Key::Ctrl;   break;
            }
        }
        break;
    case EvtType::KeyRelease:
        if (arg < 128)
        {
            isPressed[arg] = false;
        }
        else
        {
            switch (arg)
            {
            case Qt::Key_Left:  isPressed[(int)Key::Left]   = false; evt.arg = (int)Key::Left;   break;
            case Qt::Key_Right: isPressed[(int)Key::Right]  = false; evt.arg = (int)Key::Right;  break;
            case Qt::Key_Up:    isPressed[(int)Key::Up]     = false; evt.arg = (int)Key::Up;     break;
            case Qt::Key_Down:  isPressed[(int)Key::Down]   = false; evt.arg = (int)Key::Down;   break;
            case Qt::Key_Shift: isPressed[(int)Key::Shift]  = false; evt.arg = (int)Key::Shift;  break;
            case Qt::Key_Alt:   isPressed[(int)Key::Alt]    = false; evt.arg = (int)Key::Alt;    break;
            case Qt::Key_Control: isPressed[(int)Key::Ctrl] = false; evt.arg = (int)Key::Ctrl;   break;
            }
        }
        break;
    default:
        break;
    }

    _events.emplace_back(std::move(evt));
}

void Graphics::callMainFunc(Window *window, QObject* thread)
{
    mainFunc(*this, thread);

    if (closeOnExit)
    {
        if (!isClosed())
        {
            QMetaObject::invokeMethod( window, "close");
        }
    }
    else
    {
        draw();
        waitUntilClosed();
    }
}

void Graphics::setClosed()
{
    std::unique_lock<std::mutex> lock(glock);

    closed  = true;

    cv.notify_all();
}

bool Graphics::isClosed()
{
    return closed;
}

void Graphics::waitUntilClosed()
{
    std::unique_lock<std::mutex> lock(glock);

    cv.wait(lock, [this]{ return closed; });
}

void Graphics::setMousePos(int x, int y)
{
    _mousePos.x = x;
    _mousePos.y = _height-y;
}

int Graphics::registerPlugin(std::function<Plugin*(QObject*)> factory)
{
    std::unique_lock<std::mutex> lock(glock);
    pendingPlugins.emplace_back(ObjectRegistryEntry{nextPluginId, factory, std::shared_ptr<Plugin>()});
    return nextPluginId++;
}

void Graphics::callPlugin(int pluginId, int arg1, int arg2, const string &arg3)
{
    std::unique_lock<std::mutex> lock(glock);
    pluginCalls.emplace_back(PluginCall{pluginId, arg1, arg2, arg3});
}

void Graphics::draw(QWidget *pd, QPainter *painter, int width, int height, int elapsed)
{
    if (uiFunc)
    {
        uiFunc(*this, pd, elapsed);
    }

    _width = width;
    _height = height;

    std::unique_lock<std::mutex> lock(glock);

    if (!pendingPlugins.empty())
    {
        for (ObjectRegistryEntry& pe : pendingPlugins)
        {
            activePlugins.push_back(std::move(pe));
            activePlugins.back().plugin.reset(activePlugins.back().factory(pd));
            postEvent(0,0,EvtType::PluginCreated, ModKey{}, 0, pe.pluginId, "Plugin Created");
        }
        pendingPlugins.clear();
    }

    if (!pluginCalls.empty()) {
        for (const PluginCall& call : pluginCalls) {
            for (ObjectRegistryEntry& pe : activePlugins) {
                if (pe.pluginId == call.pluginId) {
                    pe.plugin->call(call.arg1, call.arg2, call.arg3);
                }
            }
        }
        pluginCalls.clear();
    }

    for (ObjectRegistryEntry& pe : activePlugins)
    {
        if (pe.plugin)
        {
            int pluginId = pe.pluginId;
            pe.plugin->update([this, pluginId](int x, int y, int arg, const std::string& data)
            { this->postEvent(x, y, EvtType::PluginMessage, ModKey(), arg, pluginId, data); });

            if (pe.plugin->shouldDelete())
            {
                postEvent(0, 0, EvtType::PluginClosed, ModKey(), 0, pluginId, "");
            }
        }
    }

    // remove dead objects
    activePlugins.erase(std::remove_if(activePlugins.begin(), activePlugins.end(), [](ObjectRegistryEntry& obj) { return !obj.plugin || obj.plugin->shouldDelete(); }), activePlugins.end());

    if (!musicFile.empty())
    {
        if (musicPlayer) {
            QObject::disconnect(musicPlayer.get(), SIGNAL(stateChanged(QMediaPlayer::State)), ((Widget*)pd)->_parent, SLOT(musicStateChanged(QMediaPlayer::State)));
        }
        musicPlayer.reset(new QMediaPlayer(pd));
        musicPlayer->setMedia(QUrl::fromLocalFile(QString::fromStdString(musicFile)));
        musicPlayer->play();
        QObject::connect(musicPlayer.get(), SIGNAL(stateChanged(QMediaPlayer::State)), ((Widget*)pd)->_parent, SLOT(musicStateChanged(QMediaPlayer::State)));
        musicFile.clear();
    }

    if (sounds.size())
    {
        for (Sound& s : sounds)
        {
            if (s.sound->play(pd))
            {
                soundCache.push_back(s);
            }
        }

        sounds.clear();
    }


    if (!stringOutput.empty())
    {
        ((Widget*)pd)->_parent->appendOutputText(stringOutput);
        stringOutput.clear();
    }

    if (finished)
    {
        if (cleared)
        {
            active.swap(queued);
            cleared = false;
        }
        else
        {
            int oldSize = active.size();
            active.resize(active.size() + queued.size());
            std::swap_ranges(active.begin()+oldSize, active.end(), queued.begin());
        }
        queued.clear();
    }

    if (!closed)
    {
        painter->begin(pd);
       // painter->setRenderHint(QPainter::Antialiasing);

            QBrush qbg = QBrush(QColor(background.r, background.g, background.b));
            painter->fillRect(0, 0, width, height, qbg);

            QMatrix m;
            m.translate(0, height);
            painter->setMatrix(m);

        for (auto& g : active)
        {
            g->draw(painter);
        }

        painter->end();
    }

    if (finished)
    {
        finished = false;
        isDrawn = true;
    }

    cv.notify_all();
}

std::vector<Event> Graphics::events()
{
    std::unique_lock<std::mutex> lock(glock);

    if (closed)
    {
        _cachedEvents.clear();
        return _cachedEvents;
    }

    if (_events.size() > 0)
    {
        append(std::move(_events), _cachedEvents);
    }

    return std::move(_cachedEvents);
}

std::string Graphics::getOutputText()
{
    std::unique_lock<std::mutex> lock(glock);

    if (closed)
    {
        return "";
    }

    string tmp = stringOutput;

    stringOutput.clear();

    return tmp;
}

bool Graphics::appendOutputText(const std::string& txt)
{
    std::unique_lock<std::mutex> lock(glock);

    if (closed)
    {
        return false;
    }

    stringOutput.append(txt);

    return true;
}


void Graphics::music(const std::string& filename)
{
    qDebug() << "graphicsMain " << QThread::currentThreadId() << endl;

    std::unique_lock<std::mutex> lock(glock);

    musicFile = findFile(filename);
}

void Graphics::play(Sound sound)
{
    std::unique_lock<std::mutex> lock(glock);

    if (closed)
    {
        return;
    }

    sounds.push_back(sound);
}

bool Graphics::draw(int minSleepTime)
{
    std::chrono::system_clock::time_point endTime;

    if (minSleepTime)
    {
        endTime = std::chrono::system_clock::now() + std::chrono::milliseconds(minSleepTime);
    }

    {
        std::unique_lock<std::mutex> lock(glock);

       // if (_events.size() > 0)
       // {
       //     _events.clear();
       //     _cachedEvents.clear();
       // }


        finished = true;

        cv.wait(lock, [this]{ return isDrawn || closed; });

        isDrawn = false;

        if (closed)
        {
            return false;
        }
    }

    if (minSleepTime)
    {
        std::this_thread::sleep_until(endTime);
    }

    return true;
}


void Graphics::clear()
{
    std::unique_lock<std::mutex> lock(glock);
    cleared = true;
}

void Graphics::addGrob(Grob *ptr)
{
    std::unique_ptr<Grob> grob(ptr);
    std::unique_lock<std::mutex> lock(glock);
    if (closed)
    {
        return;
    }
    queued.emplace_back(std::move(grob));
}

void Graphics::snapShot(Image &image)
{
    std::unique_lock<std::mutex> lock(glock);

    image.set(_width, _height, background);

    QPainter painter(image.pixmap.get());

    QMatrix m;
    m.translate(0, _height);
    painter.setMatrix(m);

    for (auto& g : active)
    {
        g->draw(&painter);
    }
}

void Graphics::image(double x, double y, const Image& image)
{
    addGrob(new GrobImage{x,y,image.pixmap});
}

void Graphics::image(double x, double y, const Image& image, int srcx, int srcy, int srcw, int srch)
{
    auto img = new GrobImage{x,y,image.pixmap};
    img->srcx = srcx;
    img->srcy = srcy;
    img->srcw = srcw;
    img->srch = srch;
    addGrob(img);
}

void Graphics::image(double x, double y, double w, double h, const Image& image)
{
    auto img = new GrobImage{x,y,image.pixmap};
    img->dstw = w;
    img->dsth = h;
    addGrob(img);
}

void Graphics::image(double x, double y, double w, double h, const Image& image, int srcx, int srcy, int srcw, int srch)
{
    auto img = new GrobImage{x,y,image.pixmap};
    img->srcx = srcx;
    img->srcy = srcy;
    img->srcw = srcw;
    img->srch = srch;
    img->dstw = w;
    img->dsth = h;
    addGrob(img);
}

void Graphics::line(double x1, double y1, double x2, double y2, Color c)
{
    addGrob(new GrobLine{x1,y1,x2,y2,c});
}

void Graphics::ellipse(double x, double y, double w, double h, Color c, Color f)
{
    addGrob(new GrobEllipse{x,y,w,h,c,f});
}

void Graphics::arc(double x, double y, double w, double h, double a, double alen, Color c, Color f)
{
    addGrob(new GrobEllipse{x,y,w,h,a,alen,1,c,f});
}

void Graphics::chord(double x, double y, double w, double h, double a, double alen, Color c, Color f)
{
    addGrob(new GrobEllipse{x,y,w,h,a,alen,2,c,f});
}

void Graphics::pie(double x, double y, double w, double h, double a, double alen, Color c, Color f)
{
    addGrob(new GrobEllipse{x,y,w,h,a,alen,3,c,f});
}

void Graphics::rect(double x, double y, double w, double h, Color c, Color f)
{
    addGrob(new GrobRect({x, y}, w, h, c, f));
}

void Graphics::polygon(std::vector<Vec2d> pts, Color border, Color fill)
{
    addGrob(new GrobPoly(pts, border, true, fill));
}

void Graphics::polyline(std::vector<Vec2d> pts, Color color)
{
    addGrob(new GrobPoly(pts, color, false));
}

void Graphics::text(double x, double y, double size, const std::string &str, Color textColor)
{
    addGrob(new GrobText(x,y,size,str,textColor));
}

void Graphics::point(double x, double y, Color c)
{
    addGrob(new GrobPoint{{x,y},c});
}

void Graphics::points(std::vector<Vec2d> pts, Color c)
{
    addGrob(new GrobPoints(pts, c));
}

std::chrono::milliseconds::rep Graphics::time()
{
    auto milliseconds_since_epoch =
            std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();

    return milliseconds_since_epoch - start_time;
}

GrobText::GrobText(double x, double y, double sz, const std::string& text, Color textColor)
{
    this->x = x;
    this->y = y;
    this->sz = sz;
    this->text = text;
    this->color = textColor;
}

void GrobText::draw(QPainter *painter)
{
    QFont f("Arial");
    f.setPixelSize(sz);
    painter->setFont(f);
    painter->setPen(QColor{color.r, color.g, color.b, color.a});
    painter->drawText(x, -y, QString(text.c_str()));
}

GrobPoints::GrobPoints(const std::vector<Vec2d>& points, Color _c)
{
    this->points.resize(points.size());

    for (unsigned int i=0;i<points.size();i++)
    {
        this->points[i].setX(points[i].x);
        this->points[i].setY(-points[i].y);
    }

    c = _c;
}

void GrobPoints::draw(QPainter *painter)
{
    QPen pen;
    pen.setWidth(20);
    pen.setColor(QColor{c.r, c.g, c.b, c.a});
    painter->setPen(pen);
    painter->drawPoints(&points[0], points.size());
}

GrobPoly::GrobPoly(const std::vector<Vec2d>& points, Color outlineColor, bool closed, Color fillColor)
{
    this->points.resize(points.size());

    for (unsigned int i=0;i<points.size();i++)
    {
        this->points[i].setX(points[i].x);
        this->points[i].setY(-points[i].y);
    }

    c = outlineColor;
    fill = fillColor;
    this->closed = closed;
}

void GrobPoly::draw(QPainter *painter)
{
    if(points.empty())
    {
        return;
    }
    painter->setPen(QColor{c.r, c.g, c.b, c.a});
    if (closed)
    {
        painter->setBrush(QBrush(QColor{fill.r, fill.g, fill.b, fill.a}));
        painter->drawPolygon(&points[0], points.size());
    }
    else
    {
        painter->drawPolyline(&points[0], points.size());
    }
}

void GrobRect::draw(QPainter *painter)
{
    painter->setPen(QColor{c.r, c.g, c.b, c.a});
    painter->setBrush(QBrush(QColor{fill.r, fill.g, fill.b, fill.a}));
    painter->drawRect(rect);
}

void GrobPoint::draw(QPainter *painter)
{
    QPen pen;
    pen.setWidth(3);
    pen.setColor(QColor{c.r, c.g, c.b, c.a});
    painter->setPen(pen);
    painter->drawPoint(p.x, -p.y);
}


void GrobLine::draw(QPainter *painter)
{
    painter->setPen(QColor{c.r, c.g, c.b, c.a});
    painter->drawLine(x1, -y1, x2, -y2);
}

void GrobEllipse::draw(QPainter *painter)
{

    switch (type)
    {
    case 0: // ellipse
        painter->setPen(QColor{c.r, c.g, c.b, c.a});
        painter->setBrush(QBrush(QColor{fill.r, fill.g, fill.b, fill.a}));
        painter->drawEllipse(x, -y-h, w, h);
        break;
    case 1: // arc
        painter->setPen(QColor{c.r, c.g, c.b, c.a});
        painter->drawArc(x,-y-h,w,h,a*180.0*16.0/M_PI,alen*180.0*16.0/M_PI);
        break;
    case 2: // chord
        painter->setPen(QColor{c.r, c.g, c.b, c.a});
        painter->setBrush(QBrush(QColor{fill.r, fill.g, fill.b, fill.a}));
        painter->drawChord(x,-y-h,w,h,a*180.0*16.0/M_PI,alen*180.0*16.0/M_PI);
        break;
    case 3: // pie
        painter->setPen(QColor{c.r, c.g, c.b, c.a});
        painter->setBrush(QBrush(QColor{fill.r, fill.g, fill.b, fill.a}));
        painter->drawPie(x,-y-h,w,h,a*180.0*16.0/M_PI,alen*180.0*16.0/M_PI);
        break;
    }
}

void GrobImage::draw(QPainter *painter)
{
    if (image)
    {
        if (srcx >= 0)
        {
            if (dsth != 0 || dstw != 0)
            {
                painter->drawPixmap(QRectF(x,-y-dsth,dstw,dsth), *image, QRect(srcx, srcy, srcw, srch));
            }
            else
            {
                painter->drawPixmap(x, -y-srch, *image, srcx, srcy, srcw, srch);
            }
        }
        else
        {
            if (dsth != 0 || dstw != 0)
            {
                painter->drawPixmap(QRectF(x,-y-dsth,dstw,dsth), *image, image->rect());
            }
            else
            {
                painter->drawPixmap(x, -y-image->height(), *image);
            }
        }
    }
}

Widget::Widget(mssm::Graphics *graphics, Window *parent)
    : QWidget(parent), _graphics(graphics), _parent(parent)
{
    lastTime = graphics->time();

    setMinimumHeight(graphics->height());
    setMinimumWidth(graphics->width());

    setFocusPolicy(Qt::StrongFocus);
    setFocus();
}

void Widget::animate()
{
    update();
}

void Widget::paintEvent(QPaintEvent *event)
{
    QPainter painter;

    auto newTime = _graphics->time();
    auto elapsed = newTime - lastTime;
    lastTime = newTime;

    _graphics->draw(this, &painter, event->rect().width(), event->rect().height(), elapsed);

    auto cursorPos = mapFromGlobal(QCursor::pos());

    _graphics->setMousePos(cursorPos.x(), cursorPos.y());

}

ModKey cvtMods(Qt::KeyboardModifiers qmods)
{
    ModKey mods = static_cast<ModKey>(
                ((qmods & Qt::ControlModifier) ? (int)ModKey::Ctrl : 0) |
                ((qmods & Qt::AltModifier) ? (int)ModKey::Alt : 0) |
                ((qmods & Qt::ShiftModifier) ? (int)ModKey::Shift : 0));

    return mods;
}

void Widget::wheelEvent(QWheelEvent *event)
{
    _graphics->handleEvent(event->x(), event->y(), EvtType::MouseWheel,
                           cvtMods(event->modifiers()),
                           event->angleDelta().y());
}


void Widget::mousePressEvent(QMouseEvent * event)
{
    _graphics->handleEvent(event->x(), event->y(), EvtType::MousePress,
                           cvtMods(event->modifiers()),
                           (int)event->button());
}

void Widget::mouseReleaseEvent(QMouseEvent * event)
{
    _graphics->handleEvent(event->x(), event->y(), EvtType::MouseRelease,
                           cvtMods(event->modifiers()),
                           (int)event->button());
}

void Widget::mouseMoveEvent(QMouseEvent * event)
{
    _graphics->handleEvent(event->x(), event->y(), EvtType::MouseMove,
                           cvtMods(event->modifiers()),
                           (int)event->buttons());
}

void Widget::keyPressEvent(QKeyEvent * event)
{
    _graphics->handleEvent(0, 0, EvtType::KeyPress,
                           cvtMods(event->modifiers()),
                           event->key());
}

void Widget::keyReleaseEvent(QKeyEvent * event)
{
    _graphics->handleEvent(0,0, EvtType::KeyRelease,
                           cvtMods(event->modifiers()),
                           event->key());
}

Window::Window(mssm::Graphics *g, std::string title)
{
    graphics = g;

    g->setInputTextFunc([this]() { return getInputText(); });

    setWindowTitle(tr(title.c_str()));

    graphicsWidget = new Widget(graphics, this);
    streamOutBox = new QPlainTextEdit;
    streamInBox = new QLineEdit;

    QGridLayout *layout = new QGridLayout;

    layout->setRowStretch(0,1);
    layout->setRowStretch(1,0);
    layout->setRowStretch(2,0);

    layout->addWidget(graphicsWidget, 0, 0);
    layout->addWidget(streamOutBox, 1, 0);
    layout->addWidget(streamInBox, 2, 0);

    streamOutBox->setMaximumBlockCount(100);

    streamInBox->hide();
    streamOutBox->hide();
    streamInBox->setEnabled(false);

    connect(streamInBox, SIGNAL(returnPressed()), this, SLOT(textEntered()));

    setLayout(layout);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), graphicsWidget, SLOT(animate()));
    timer->start(24);
}

void Window::appendOutputText(const std::string& txt)
{
    if (graphics->isClosed())
    {
        return;
    }

    if (streamOutBox->isHidden())
    {
        streamOutBox->show();
    }

    streamOutBox->moveCursor(QTextCursor::End);
    streamOutBox->insertPlainText(QString::fromStdString(txt));
    streamOutBox->moveCursor(QTextCursor::End);
}

std::string Window::getInputText()
{
    if (graphics->isClosed())
    {
        return "";
    }

    if (!usingStreamIO)
    {
        usingStreamIO = true;
        QMetaObject::invokeMethod( streamInBox, "show");
    }

    QMetaObject::invokeMethod( streamInBox, "setEnabled", Q_ARG(bool, true));
    QMetaObject::invokeMethod( streamInBox, "setFocus");

    std::unique_lock<std::mutex> lock(wlock);

    cv.wait(lock, [this]{ return hasEnteredText; });

    string str;

    if (hasEnteredText)
    {
        std::swap(str, enteredText);
        hasEnteredText = false;
    }

    return str;
}

void Window::textEntered()
{
    auto str = streamInBox->text().toStdString();

    streamInBox->clear();

    std::unique_lock<std::mutex> lock(wlock);

    enteredText.append(str);
    enteredText.append("\n");

    hasEnteredText = true;

    streamInBox->setEnabled(false);
    graphicsWidget->setFocus();

    cv.notify_all();
}

void Window::musicStateChanged(QMediaPlayer::State state)
{
    qDebug() << "Music State Changed: " << static_cast<int>(state) << " Thread: " << QThread::currentThreadId();
    graphics->postEvent(0, 0, EvtType::PluginMessage, ModKey{}, state, 0, "MusicPlayer");
}

namespace mssm
{

// takes rgb values 0-1
Color doubleToColor(double r, double g, double b)
{
    Color out;
    r *= 255;
    g *= 255;
    b *= 255;
    out.r = r < 0 ? 0 : (r > 255 ? 255 : r);
    out.g = g < 0 ? 0 : (g > 255 ? 255 : g);
    out.b = b < 0 ? 0 : (b > 255 ? 255 : b);
    return out;
}

// h   0-360
// s   0-1
// v   0-1
Color hsv2rgb(double h, double s, double v)
{
    if (s <= 0.0) // < 0 shouldn't happen...
    {
        return doubleToColor(v,v,v);
    }

    double hh = (h >= 360 ? 0.0 : h) / 60.0;
    int    i  = hh;
    double ff = hh - i;

    double p = v * (1.0 - s);
    double q = v * (1.0 - (s * ff));
    double t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
    case 0:
        return doubleToColor(v,t,p);
    case 1:
        return doubleToColor(q,v,p);
    case 2:
        return doubleToColor(p,v,t);
    case 3:
        return doubleToColor(p,q,v);
    case 4:
        return doubleToColor(t,p,v);
    case 5:
    default:
        return doubleToColor(v,p,q);
    }
}

} // namespace mssm

