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

#include <deque>

#ifndef _MSC_VER
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif

#ifdef INCLUDE_SOUND
#include "soloud.h"
#include "soloud_wav.h"
#endif

#ifndef _MSC_VER
#pragma GCC diagnostic pop
#endif 

#include "vec2d.h"
#include "rand.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

class QNanoPainter;
class GLFWwindow;
class NVGcontext;

namespace mssm
{

enum class HAlign {
    left   = 1<<0,
    center = 1<<1,
    right  = 1<<2
};

enum class VAlign {
    top      = 1<<3,
    center   = 1<<4,
    bottom   = 1<<5,
    baseline = 1<<6,
};

/* Defines for GLFW Function keys, so we don't have to include all of glfw in graphics.h  */
/* hope these don't change on us... */
#define fwdDefGLFW_KEY_ESCAPE             256
#define fwdDefGLFW_KEY_ENTER              257
#define fwdDefGLFW_KEY_TAB                258
#define fwdDefGLFW_KEY_BACKSPACE          259
#define fwdDefGLFW_KEY_INSERT             260
#define fwdDefGLFW_KEY_DELETE             261
#define fwdDefGLFW_KEY_RIGHT              262
#define fwdDefGLFW_KEY_LEFT               263
#define fwdDefGLFW_KEY_DOWN               264
#define fwdDefGLFW_KEY_UP                 265
#define fwdDefGLFW_KEY_PAGE_UP            266
#define fwdDefGLFW_KEY_PAGE_DOWN          267
#define fwdDefGLFW_KEY_HOME               268
#define fwdDefGLFW_KEY_END                269
#define fwdDefGLFW_KEY_CAPS_LOCK          280
#define fwdDefGLFW_KEY_SCROLL_LOCK        281
#define fwdDefGLFW_KEY_NUM_LOCK           282
#define fwdDefGLFW_KEY_PRINT_SCREEN       283
#define fwdDefGLFW_KEY_PAUSE              284
#define fwdDefGLFW_KEY_F1                 290
#define fwdDefGLFW_KEY_F2                 291
#define fwdDefGLFW_KEY_F3                 292
#define fwdDefGLFW_KEY_F4                 293
#define fwdDefGLFW_KEY_F5                 294
#define fwdDefGLFW_KEY_F6                 295
#define fwdDefGLFW_KEY_F7                 296
#define fwdDefGLFW_KEY_F8                 297
#define fwdDefGLFW_KEY_F9                 298
#define fwdDefGLFW_KEY_F10                299
#define fwdDefGLFW_KEY_F11                300
#define fwdDefGLFW_KEY_F12                301
#define fwdDefGLFW_KEY_F13                302
#define fwdDefGLFW_KEY_F14                303
#define fwdDefGLFW_KEY_F15                304
#define fwdDefGLFW_KEY_F16                305
#define fwdDefGLFW_KEY_F17                306
#define fwdDefGLFW_KEY_F18                307
#define fwdDefGLFW_KEY_F19                308
#define fwdDefGLFW_KEY_F20                309
#define fwdDefGLFW_KEY_F21                310
#define fwdDefGLFW_KEY_F22                311
#define fwdDefGLFW_KEY_F23                312
#define fwdDefGLFW_KEY_F24                313
#define fwdDefGLFW_KEY_F25                314
#define fwdDefGLFW_KEY_KP_0               320
#define fwdDefGLFW_KEY_KP_1               321
#define fwdDefGLFW_KEY_KP_2               322
#define fwdDefGLFW_KEY_KP_3               323
#define fwdDefGLFW_KEY_KP_4               324
#define fwdDefGLFW_KEY_KP_5               325
#define fwdDefGLFW_KEY_KP_6               326
#define fwdDefGLFW_KEY_KP_7               327
#define fwdDefGLFW_KEY_KP_8               328
#define fwdDefGLFW_KEY_KP_9               329
#define fwdDefGLFW_KEY_KP_DECIMAL         330
#define fwdDefGLFW_KEY_KP_DIVIDE          331
#define fwdDefGLFW_KEY_KP_MULTIPLY        332
#define fwdDefGLFW_KEY_KP_SUBTRACT        333
#define fwdDefGLFW_KEY_KP_ADD             334
#define fwdDefGLFW_KEY_KP_ENTER           335
#define fwdDefGLFW_KEY_KP_EQUAL           336
#define fwdDefGLFW_KEY_LEFT_SHIFT         340
#define fwdDefGLFW_KEY_LEFT_CONTROL       341
#define fwdDefGLFW_KEY_LEFT_ALT           342
#define fwdDefGLFW_KEY_LEFT_SUPER         343
#define fwdDefGLFW_KEY_RIGHT_SHIFT        344
#define fwdDefGLFW_KEY_RIGHT_CONTROL      345
#define fwdDefGLFW_KEY_RIGHT_ALT          346
#define fwdDefGLFW_KEY_RIGHT_SUPER        347
#define fwdDefGLFW_KEY_MENU               348

enum class Key
{
    Left  = fwdDefGLFW_KEY_LEFT,
    Right = fwdDefGLFW_KEY_RIGHT,
    Up    = fwdDefGLFW_KEY_UP,
    Down  = fwdDefGLFW_KEY_DOWN,
    LeftShift = fwdDefGLFW_KEY_LEFT_SHIFT,
    LeftCtrl  = fwdDefGLFW_KEY_LEFT_CONTROL,
    LeftAlt   = fwdDefGLFW_KEY_LEFT_ALT,
    RightShift = fwdDefGLFW_KEY_RIGHT_SHIFT,
    RightCtrl  = fwdDefGLFW_KEY_RIGHT_CONTROL,
    RightAlt   = fwdDefGLFW_KEY_RIGHT_ALT,
    ESC   = fwdDefGLFW_KEY_ESCAPE,
    ENTER = fwdDefGLFW_KEY_ENTER,
    PageUp = fwdDefGLFW_KEY_PAGE_UP,
    PageDown = fwdDefGLFW_KEY_PAGE_DOWN,
};

enum class EvtType
{
    MousePress,   // arg = button,  x and y = mouse pos
    MouseRelease, // arg = button,  x and y = mouse pos
    MouseMove,    // arg = button,  x and y = mouse pos
    MouseWheel,   // arg = delta, x and y = mouse pos
    KeyPress,     // arg = key
    KeyRelease,   // arg = key
    MusicEvent,   // arg:  0 = stopped,  1 = playing,  2 = paused
    WindowResize  // x and y => width and height.  arg = 0 windowed  arg = 1 fullscreen
};

enum class ModKey
{
    Shift = 1 << 0,
    Alt   = 1 << 1,
    Ctrl  = 1 << 2
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
    unsigned int toUIntRGBA() const { return a & (b << 8) & (g << 16) & (r << 24); }
    int toIntRGB() const { return b & (g << 8) & (r << 16); }
};
#undef TRANSPARENT

constexpr Color TRANSPARENT(0,0,0,0);
constexpr Color TRANS(0,0,0,0);
constexpr Color WHITE(255,255,255);
constexpr Color GREY(128,128,128);
constexpr Color BLACK(0,0,0);
constexpr Color RED(255,0,0);
constexpr Color GREEN(0,255,0);
constexpr Color BLUE(0,0,255);
constexpr Color YELLOW(255,255,0);
constexpr Color PURPLE(255,0,255);
constexpr Color CYAN(0,255,255);
constexpr Color ORANGE(255,165,0);
constexpr Color LTGREY(211,211,211);

Color hsv2rgb(double h, double s, double v);
void  rgb2hsv(Color c, double &h, double &s, double &v);

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
    Vec2d mousePos() const { return Vec2d(x, y); }
    bool hasCtrl()     { return static_cast<int>(mods) & static_cast<int>(ModKey::Ctrl);  }
    bool hasAlt()      { return static_cast<int>(mods) & static_cast<int>(ModKey::Alt);   }
    bool hasShift()    { return static_cast<int>(mods) & static_cast<int>(ModKey::Shift); }
    char key()         { return char(arg); }
    int  mouseButton() { return arg; }
};

std::ostream& operator<<(std::ostream& os, const Event& evt);

class Grob;

class ImageInternal {
    NVGcontext* vg;
    int vgImageIdx;
    int width{0};
    int height{0};
    Color* pixels{nullptr};
public:
    ImageInternal(NVGcontext* vg, int idx, int width, int height, Color* cached);
   ~ImageInternal();
private:
    void freeCachedPixels();
    void updatePixels();
    void setPixel(int x, int y, Color c) {
       pixels[y*width+x] = c;
    }
    Color getPixel(int x, int y) {
        return pixels[y*width+x];
    }
    friend class Image;
    friend class Graphics;
};

class Graphics;

class Image
{
private:
    NVGcontext* vg;
    std::shared_ptr<ImageInternal> pixmap;
public:
    Image(mssm::Graphics& g);
    Image(mssm::Graphics& g, int width, int height, Color c, bool cachePixels = false);
    Image(mssm::Graphics& g, const std::string& filename, bool cachePixels = false);
    void set(int width, int height, Color c, bool cachePixels = false);
    void load(const std::string& fileName, bool cachePixels = false);
    void save(const std::string& pngFileName);
    Color* pixels() { return pixmap->pixels; } // changes won't take effect until updatePixels
    void  setPixel(int x, int y, Color c) { pixmap->setPixel(x, y, c); } // won't take effect until updatePixels
    Color getPixel(int x, int y)          { return pixmap->getPixel(x, y); }
    void updatePixels() { pixmap->updatePixels(); }
    int width() const;
    int height() const;
    friend class Graphics;
};

#ifdef INCLUDE_SOUND
class SoundInternal
{
private:
    std::string filename;
    SoLoud::Wav wave;
    SoLoud::Soloud& player;
public:
    SoundInternal(SoLoud::Soloud& player, const std::string& filename);
   ~SoundInternal();
private:
    bool play();  // returns true when first creating QSoundEffect
    int  status();
    void release();
    friend class Graphics;
    friend class Sound;
};
#else
class SoundInternal
{

};
#endif

class Sound
{
private:
    std::shared_ptr<SoundInternal> sound;
public:
    Sound(mssm::Graphics& g, const std::string& filename);
    friend class Graphics;
};

class TextExtents {
public:
    float fontAscent;
    float fontDescent;
    float fontHeight;
    float textHeight;
    float textWidth;
    float textAdvance;
};

class Graphics
{
private:
    GLFWwindow* window{nullptr};

#ifdef _WIN32
    HDC hdc;
#endif

    NVGcontext* vg{nullptr};

    Rand rnd;
    std::function<void (Graphics&)> mainFunc;

#ifdef INCLUDE_SOUND
    SoLoud::Soloud                     soundPlayer;
#endif

    std::string                        musicFile;

    int fontRegular;
    int fontBold;
    int fontLight;

    std::vector<Event> _events;
    std::vector<Event> _cachedEvents;
    std::string title;

    bool        closed{false};
    bool        finished{false};
    bool        isDrawn{false};
    bool        cleared{false};

    bool        requestToggleFullScreen{false};

    std::chrono::microseconds::rep start_time;
    std::chrono::steady_clock::time_point lastDrawTime;
    std::chrono::microseconds::rep elapsed;

    int         currentWidth{100};
    int         currentHeight{100};

    bool        gotResizeEvent{false};

    int         windowedX{0};
    int         windowedY{0};
    int         windowedWidth;
    int         windowedHeight;

    mssm::Color background;
    std::vector<bool> isPressed;
    //std::vector<bool> wasPressed;
    std::string stringOutput;
    std::function<std::string()> getInputText;
    Vec2d       _mousePos; // mouse pos at time of last screen repaint

    void postEvent(int x, int y, EvtType evtType, ModKey mods, int arg, int pluginId = 0, const std::string& data = std::string());

    std::vector<std::shared_ptr<ImageInternal>> keepImages;

public:
    Graphics(std::string title, int width, int height,
             std::function<void (Graphics&)> mainThreadFunc = nullptr);


    ~Graphics();
public:
    std::stringstream cout;
    std::stringstream cerr;
    std::deque<std::string> cerrLines;

    NVGcontext* vgContext() { return vg; }

#ifdef INCLUDE_SOUND
    SoLoud::Soloud& getSoundPlayer() { return soundPlayer; }
#endif

    std::chrono::milliseconds::rep time();

    void test();

    void waitUntilClosed();

    double elapsedMs() { return elapsed/1000.0; }
   // void   callPlugin(int pluginId, int arg1, int arg2, const std::string& arg3);

    int    width()  { return currentWidth; }
    int    height() { return currentHeight; }

    Vec2d  mousePos();

    void   setBackground(Color c) { background = c; }

    void   line(Vec2d p1, Vec2d p2, Color c = WHITE);
    void   ellipse(Vec2d center, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
    void   arc(Vec2d center, double w, double h, double a, double alen, Color c = WHITE);
    void   chord(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
    void   pie(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
    void   rect(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
    void   polygon(std::vector<Vec2d> pts, Color border, Color fill = TRANSPARENT);
    void   polyline(std::vector<Vec2d> pts, Color color);
    void   text(Vec2d pos, double size, const std::string& str, Color textColor = WHITE, HAlign hAlign = HAlign::left, VAlign vAlign = VAlign::baseline);

    void   textExtents(double size, const std::string& str, TextExtents& extents);

    void   point(Vec2d pos, Color c);
    void   points(std::vector<Vec2d> pts, Color c);

    void   image(Vec2d pos, const Image& img);
    void   image(Vec2d pos, const Image& img, Vec2d src, int srcw, int srch);
    void   image(Vec2d pos, double w, double h, const Image& img);
    void   image(Vec2d pos, double w, double h, const Image& img, Vec2d src, int srcw, int srch);

    void   imageC(Vec2d center, double angle, const Image& img);
    void   imageC(Vec2d center, double angle, const Image& img, Vec2d src, int srcw, int srch);
    void   imageC(Vec2d center, double angle, double w, double h, const Image& img);
    void   imageC(Vec2d center, double angle, double w, double h, const Image& img, Vec2d src, int srcw, int srch);

    void   snapShot(Image& image);

    void   play(Sound sound);
    void   music(const std::string& filename);

    void   clear();
    bool   wasCleared() { return cleared; } // true until the next draw

    bool   draw();
    void   drawFromStream(std::stringstream& ss, std::deque<std::string>& lines, Vec2d start, Color c);

    bool   isClosed();

    double timeMs();

    bool   isKeyPressed(int c) { return isPressed[c]; }
    bool   isKeyPressed(Key k);

    std::vector<Event> events();

    int    randomInt(int minVal, int maxVal) { return rnd.randomInt(minVal, maxVal); }
    double randomDouble(double minVal, double maxVal) { return rnd.randomDouble(minVal, maxVal); }
    bool   randomTrue(double pct) { return rnd.randomTrue(pct); }

    std::string currentPath(const std::string& file = "");
    std::string programName();

private:
  //  void draw(/*QWidget *pd, */QNanoPainter *painter, int width, int height, double uiFuncElapsed);
    std::string getTitle() { return title; }
    void setClosed();


    std::string getOutputText();
    bool appendOutputText(const std::string& txt);
    void setInputTextFunc(std::function<std::string()> func) { getInputText = func; }

    void setMousePos(int x, int y);

    void toggleFullScreen() { requestToggleFullScreen = true; }

    friend void key(GLFWwindow* window, int key, int scancode, int action, int mods);
    friend void mousePosCallback(GLFWwindow* window, double x, double y);
    friend void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    friend void scrollWheelCallback(GLFWwindow* window, double /*sx*/, double sy);
    friend void windowSizeCallback(GLFWwindow* window,int width,int height);
};


}

#endif // GROBS_H
