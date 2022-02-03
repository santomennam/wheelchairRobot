#ifndef GROBS_H
#define GROBS_H

#define _USE_MATH_DEFINES
#include <cmath>

#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <sstream>
#include <random>

#include "vec2d.h"
#include "plugin.h"

// graphical objects

class QPainter;
class QPointF;
class QPaintDevice;

class Window;
class Widget;
class QPixmap;
class QMediaPlayer;
class QSoundEffect;
class QObject;
class QWidget;
class QObject;
class QWidget;

Q_DECLARE_METATYPE(std::string)

enum class Key
{
    Left  = 17,
    Right = 18,
    Up    = 19,
    Down  = 20,
    Shift = 29,
    Ctrl  = 30,
    Alt   = 31,
    ESC   = 16777216,
    ENTER = 16777220
};

enum class EvtType
{
    MousePress,   // arg = button,  x and y = mouse pos
    MouseRelease, // arg = button,  x and y = mouse pos
    MouseMove,    // arg = button,  x and y = mouse pos
    MouseWheel,   // arg = delta, x and y = mouse pos
    KeyPress,     // arg = key
    KeyRelease,   // arg = key
    PluginCreated,
    PluginClosed,
    PluginMessage,
};

enum class ModKey
{
    Shift = 1 << 0,
    Alt   = 1 << 1,
    Ctrl  = 1 << 2
};


namespace mssm
{
    class ObjectRegistryEntry
    {
    public:
        int pluginId;
        std::function<Plugin*(QObject*)> factory;
        std::shared_ptr<Plugin>          plugin;
    };

    class PluginCall
    {
    public:
        int pluginId;
        int arg1;
        int arg2;
        std::string arg3;
    };

    class Color
    {
    public:
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char a{255};
    public:
        constexpr Color(int c) : r((c >> 16)&0xFF), g((c >> 8)&0xFF), b(c&0xFF), a(0xFF) {}
        constexpr Color()  : r(0), g(0), b(0), a(255) {}
        constexpr Color(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a = 255)  : r(_r), g(_g), b(_b), a(_a) {}
    };

    constexpr Color TRANSPARENT(0,0,0,0);
    constexpr Color WHITE(255,255,255);
    constexpr Color GREY(128,128,128);
    constexpr Color BLACK(0,0,0);
    constexpr Color RED(255,0,0);
    constexpr Color GREEN(0,255,0);
    constexpr Color BLUE(0,0,255);
    constexpr Color YELLOW(255,255,0);
    constexpr Color PURPLE(255,0,255);
    constexpr Color CYAN(0,255,255);

    Color hsv2rgb(double h, double s, double v);

    class Event
    {
    public:
        EvtType evtType;
        int     x;
        int     y;
        ModKey  mods;
        int     arg;
        int     pluginId;
        std::string data;
    public:
        bool hasCtrl()     { return static_cast<int>(mods) & static_cast<int>(ModKey::Ctrl);  }
        bool hasAlt()      { return static_cast<int>(mods) & static_cast<int>(ModKey::Alt);   }
        bool hasShift()    { return static_cast<int>(mods) & static_cast<int>(ModKey::Shift); }
        char key()         { return arg; }
        int  mouseButton() { return arg; }
    };

    std::ostream& operator<<(std::ostream& os, const Event& evt);

    class Grob;


    class Image
    {
    private:
        std::shared_ptr<QPixmap> pixmap;
    public:
        Image() {}
        Image(int width, int height, Color c);
        Image(const std::string& filename);
        void set(const std::vector<Color>& pixels, int width, int height);
        void set(int width, int height, Color c);
        void load(const std::string& fileName);
        void save(const std::string& pngFileName);
        int width();
        int height();
        friend class Graphics;
    };

    class SoundInternal
    {
    private:
        std::string                   filename;
        std::shared_ptr<QSoundEffect> sound;
    public:
        SoundInternal(const std::string& filename);
    private:
        bool play(QObject* parent);  // returns true when first creating QSoundEffect
        int  status();
        void release();
        friend class Graphics;
        friend class Sound;
    };

    class Sound
    {
    private:
        std::shared_ptr<SoundInternal> sound;
    public:
        Sound(const std::string& filename);
        friend class Graphics;
    };

    class Graphics
    {
    private:
        std::random_device randDevice;
        std::mt19937 mersenneTwister;
        std::function<void (Graphics&, QObject*)> mainFunc;
        std::function<void (Graphics&, QObject*, int)> uiFunc;
        std::mutex glock;
        std::condition_variable cv;
        std::vector<std::unique_ptr<Grob>> queued;
        std::vector<std::unique_ptr<Grob>> active;

        std::vector<Sound>                 sounds;
        std::vector<Sound>                 soundCache;

        std::string                        musicFile;
        std::unique_ptr<QMediaPlayer>      musicPlayer;

        int                                nextPluginId{1};
        std::vector<ObjectRegistryEntry>   pendingPlugins;
        std::vector<ObjectRegistryEntry>   activePlugins;
        std::vector<PluginCall>            pluginCalls;

        std::vector<Event> _events;
        std::vector<Event> _cachedEvents;
        std::string title;
        bool        closeOnExit{false};
        bool        closed{false};
        bool        finished{false};
        bool        isDrawn{false};
        bool        cleared{false};
        std::chrono::milliseconds::rep start_time;
        int         _width{100};
        int         _height{100};
        mssm::Color background;
        std::vector<bool> isPressed;
        std::vector<bool> wasPressed;
        std::string stringOutput;
        std::function<std::string()> getInputText;
        Vec2d       _mousePos; // mouse pos at time of last screen repaint
        // post event should only be called by the ui thread!
        void postEvent(int x, int y, EvtType evtType, ModKey mods, int arg, int pluginId = 0, const std::string& data = std::string());

    public:
        Graphics(std::string title, int width, int height,
                 std::function<void (Graphics&, QObject*)> mainThreadFunc,
                 std::function<void (Graphics&, QObject*, int)> uiThreadFunc = std::function<void (Graphics&, QObject*, int)>());
        Graphics(std::string title, int width, int height,
                 std::function<void (Graphics&)> mainThreadFunc);

        ~Graphics();

        void handleEvent(int x, int y, EvtType evtType, ModKey mods, int arg, int pluginId = 0, const std::string& data = std::string());
    public:
        std::istream in;
        std::ostream out;

        int    registerPlugin(std::function<Plugin*(QObject*)> factory); // returns pluginId for use by callPlugin
        void   callPlugin(int pluginId, int arg1, int arg2, const std::string& arg3);

        int    width()  { return _width; }
        int    height() { return _height; }

        Vec2d  mousePos() { return _mousePos; }
        void   mousePos(int& x, int& y) { x = _mousePos.x; y = _mousePos.y; }

        void   setBackground(Color c) { background = c; }

        void   line(double x1, double y1, double x2, double y2, Color c = WHITE);
        void   line(Vec2d p1, Vec2d p2, Color c = WHITE) { line(p1.x, p1.y, p2.x, p2.y, c); }

        void   ellipse(double x, double y, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
        void   ellipse(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT) { ellipse(corner.x, corner.y, w, h, c, f); }
        void   ellipseC(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT) { ellipse(corner.x-w/2, corner.y-h/2, w, h, c, f); }

        void   arc(double x, double y, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
        void   arc(Vec2d corner, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT) { arc(corner.x, corner.y, w, h, a, alen, c, f); }

        void   chord(double x, double y, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
        void   chord(Vec2d corner, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT) { chord(corner.x, corner.y, w, h, a, alen, c, f); }

        void   pie(double x, double y, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
        void   pie(Vec2d corner, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT) { pie(corner.x, corner.y, w, h, a, alen, c, f); }

        void   rect(double x, double y, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
        void   rect(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT) { rect(corner.x, corner.y, w, h, c, f); }

        void   polygon(std::vector<Vec2d> pts, Color border, Color fill = TRANSPARENT);
        void   polyline(std::vector<Vec2d> pts, Color color);

        void   text(double x, double y, double size, const std::string& str, Color textColor = WHITE);
        void   text(Vec2d pos, double size, const std::string& str, Color textColor = WHITE) { text(pos.x, pos.y, size, str, textColor); }

        void   point(double x, double y, Color c);
        void   point(Vec2d pos, Color c) { point(pos.x, pos.y, c); }

        void   points(std::vector<Vec2d> pts, Color c);

        void   snapShot(Image& image);

        void   image(double x, double y, const Image& image);
        void   image(Vec2d pos, const Image& img) { image(pos.x, pos.y, img); }
        void   image(double x, double y, const Image& image, int srcx, int srcy, int srcw, int srch);
        void   image(Vec2d pos, const Image& img, int srcx, int srcy, int srcw, int srch) { image(pos.x, pos.y, img, srcx, srcy, srcw, srch); }

        void   image(double x, double y, double w, double h, const Image& image);
        void   image(Vec2d pos, double w, double h, const Image& img) { image(pos.x, pos.y, w, h, img); }
        void   image(double x, double y, double w, double h, const Image& image, int srcx, int srcy, int srcw, int srch);
        void   image(Vec2d pos, double w, double h, const Image& img, int srcx, int srcy, int srcw, int srch) { image(pos.x, pos.y, w, h, img, srcx, srcy, srcw, srch); }

        void   play(Sound sound);
        void   music(const std::string& filename);

        void   clear();
        bool   wasCleared() { return cleared; } // true until the next draw

        bool   draw(int delay = 0);

        void   setCloseOnExit(bool close = true) { closeOnExit = close; }
        bool   isClosed();
        void   waitUntilClosed();

        std::chrono::milliseconds::rep time();

        bool   isKeyPressed(char c) { return isPressed[c]; }
        bool   isKeyPressed(Key k);

        std::vector<Event> events();

        int    randomInt(int minVal, int maxVal);
        double randomDouble(double minVal, double maxVal);
        bool   randomTrue(double pct);

        std::string currentPath(const std::string& file = "");
        std::string programName();

    private:
        void draw(QWidget *pd, QPainter *painter, int width, int height, int elapsed);
        std::string getTitle() { return title; }
        void setClosed();
        void callMainFunc(Window *window, QObject *thread);
        void addGrob(Grob* ptr);

        std::string getOutputText();
        bool appendOutputText(const std::string& txt);
        void setInputTextFunc(std::function<std::string()> func) { getInputText = func; }

        void setMousePos(int x, int y);

        friend void helper(Graphics* g, Window *window, QObject* thread);
        friend class ::Widget;
        friend class ::Window;
    };

    std::string findFile(const std::string& filename);
}

#endif // GROBS_H
