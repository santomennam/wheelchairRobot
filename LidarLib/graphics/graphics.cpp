#define _USE_MATH_DEFINES
#define NOMINMAX

#pragma warning( disable : 4244 )

#include "graphics.h"
//#include "window.h"

#include <iostream>

#include <functional>
//#include <thread>
#include <chrono>
//#include <mutex>
//#include <condition_variable>
#include <random>
#include <cmath>
//#include <filesystem>
#include <vector>
#include <iterator>
#include <utility>
#include <iomanip>

#include <stdio.h>

#include "paths.h"




#ifdef NANOVG_GLEW
#	include <GL/glew.h>
#endif
#ifdef __APPLE__
#	define GLFW_INCLUDE_GLCOREARB
#endif
#define GLFW_INCLUDE_GLEXT
#include "GLFW/glfw3.h"

#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include "GLFW/glfw3native.h"
#endif


#include "nanovg.h"
#define NANOVG_GL3_IMPLEMENTATION
#include "nanovg_gl.h"

//#include "nanovg.h"
//#define NANOVG_GL3_IMPLEMENTATION
//#include "nanovg_gl.h"

//#include "GL/wglew.h"

//https://stackoverflow.com/questions/47402766/switching-between-windowed-and-full-screen-in-opengl-glfw-3-2/47462358

#include "stb_image.h"

class QNanoPainter {
public:
    NVGcontext* vg;
};

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
    int_type underflow() override;

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

    std::copy(_source.begin(), _source.begin()+int(count), dst);

    _source.erase(_source.begin(), _source.begin()+int(count));

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
    int_type overflow(int_type ch) override;
    int sync() override;

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

// thanks to: https://stackoverflow.com/users/514130/shmo

GLFWmonitor* get_current_monitor(GLFWwindow *window)
{
    int nmonitors, i;
    int wx, wy, ww, wh;
    int mx, my, mw, mh;
    int overlap, bestoverlap;
    GLFWmonitor *bestmonitor;
    GLFWmonitor **monitors;
    const GLFWvidmode *mode;

    bestoverlap = 0;
    bestmonitor = NULL;

    glfwGetWindowPos(window, &wx, &wy);
    glfwGetWindowSize(window, &ww, &wh);
    monitors = glfwGetMonitors(&nmonitors);

    for (i = 0; i < nmonitors; i++) {
        mode = glfwGetVideoMode(monitors[i]);
        glfwGetMonitorPos(monitors[i], &mx, &my);
        mw = mode->width;
        mh = mode->height;

        overlap =
                max(0, min(wx + ww, mx + mw) - max(wx, mx)) *
                max(0, min(wy + wh, my + mh) - max(wy, my));

        if (bestoverlap < overlap) {
            bestoverlap = overlap;
            bestmonitor = monitors[i];
        }
    }

    if (!bestmonitor) {
        // not sure that this could even happen, but let's be safe
        bestmonitor = glfwGetPrimaryMonitor();
    }

    return bestmonitor;
}


namespace mssm
{

Sound::Sound(mssm::Graphics& g, const string &filename)
{
    auto fpath = Paths::findAsset(filename);
    if (!fpath.empty()) {
#ifdef INCLUDE_SOUND
        sound = std::make_shared<SoundInternal>(g.getSoundPlayer(), fpath);
#else
        cout << "Warning:  Attempt to use sound, but sound not included in library" << endl;
#endif
    }
    else
    {
        g.cerr << "Error loading sound " << filename << endl;
    }
}

Image::Image(Graphics& g, int width, int height, Color c, bool cachePixels)
    : vg{g.vgContext()}
{
    set(width, height, c, cachePixels);
}

Image::Image(Graphics& g, const std::string& filename, bool cachePixels)
    : vg{g.vgContext()}
{
    load(filename, cachePixels);
}

int nvgCreateImageCache(NVGcontext* ctx, unsigned char*& cachedImg, const char* filename, int imageFlags)
{
    int w, h, n, image;
    stbi_set_unpremultiply_on_load(1);
    stbi_convert_iphone_png_to_rgb(1);
    cachedImg = stbi_load(filename, &w, &h, &n, 4);
    if (cachedImg == NULL) {
//		printf("Failed to load %s - %s\n", filename, stbi_failure_reason());
        return 0;
    }
    image = nvgCreateImageRGBA(ctx, w, h, imageFlags, cachedImg);
    //stbi_image_free(img);
    return image;
}

void Image::load(const std::string& fileName, bool cachePixels)
{
    auto fpath = Paths::findAsset(fileName);
    unsigned char* cachedImage = nullptr;
    int idx;

    if (cachePixels) {
        idx = nvgCreateImageCache(vg, cachedImage, fpath.c_str(), 0);
    }
    else {
        idx = nvgCreateImage(vg, fpath.c_str(), 0);
    }

    if (idx) {
        int w;
        int h;
        nvgImageSize(vg, idx, &w, &h);
        pixmap = std::make_shared<ImageInternal>(vg, idx, w, h, reinterpret_cast<Color*>(cachedImage));
    }
    else {
        if (cachedImage) {
            stbi_image_free(cachedImage);
        }
        set(20,20,RED,cachePixels);
    }
}

//void Image::save(const string &pngFileName)
//{
//    //    QFile file(pngFileName.c_str());
//    //    file.open(QIODevice::WriteOnly);
//    //    pixmap->save(&file, "PNG");
//}

Image::Image(Graphics &g)
    : vg{g.vgContext()}
{
}

//void Image::set(const Color *pixels, int w, int h)
//{
//    if (width() == w && height() ==  h) {
//        nvgUpdateImage(vg, pixmap->vgImageIdx, reinterpret_cast<const unsigned char*>(pixels));
//    }
//    else {
//        int idx = nvgCreateImageRGBA(vg, w, h, NVG_IMAGE_NEAREST, reinterpret_cast<const unsigned char*>(pixels));
//        pixmap = std::make_shared<ImageInternal>(vg, idx, w, h);
//    }
//}

void Image::set(int width, int height, Color c, bool cachePixels)
{
    Color* pixels = reinterpret_cast<Color*>(malloc(width * height * sizeof(Color)));

    for (int i = 0; i < width*height; i++) {
        pixels[i] = c;
    }

    int idx = nvgCreateImageRGBA(vg, width, height, NVG_IMAGE_NEAREST, reinterpret_cast<const unsigned char*>(pixels));

    pixmap = std::make_shared<ImageInternal>(vg, idx, width, height, cachePixels ? pixels : nullptr);

    if (!cachePixels) {
        free(pixels);
    }
}

int Image::width() const
{
    return pixmap ? pixmap->width : 0;
}

int Image::height() const
{
    return pixmap ? pixmap->height : 0;
}

#ifdef INCLUDE_SOUND
SoundInternal::SoundInternal(SoLoud::Soloud &player, const std::string& _filename)
    : filename(_filename), player{player}
{
    wave.load(filename.c_str());
}

SoundInternal::~SoundInternal()
{

}

void SoundInternal::release()
{
}

bool SoundInternal::play(/*QObject* parent*/)
{
    player.play(wave);
    bool created = true;

    //    if (!sound)
    //    {
    //        sound = std::make_shared<QSoundEffect>(parent);
    //        created = true;
    //    }

    //    switch (sound->status())
    //    {
    //    case QSoundEffect::Status::Null:
    //        sound->setSource(QUrl::fromLocalFile(QString::fromStdString(filename)));
    //        sound->setLoopCount(1);
    //        sound->play();
    //        break;
    //    case QSoundEffect::Status::Ready:
    //        sound->setLoopCount(1);
    //        sound->play();
    //        break;
    //    case QSoundEffect::Status::Loading:
    //        // cout << "Still Loading Sound!" << endl;
    //        break;
    //    case QSoundEffect::Status::Error:
    //        cout << "Error playing sound: " << filename << endl;
    //        break;
    //    }

    return created;
}

int SoundInternal::status()
{
    //    if (!sound)
    //    {
    return 0;
    //    }
    //    return int(sound->status());
}
#endif

ImageInternal::ImageInternal(NVGcontext* vg, int idx, int width, int height, Color *cached)
    : vg{vg}, vgImageIdx{idx}, width{width}, height{height}, pixels{cached}
{
}

ImageInternal::~ImageInternal()
{
    if (vgImageIdx) {
        nvgDeleteImage(vg, vgImageIdx);
    }
}

void ImageInternal::freeCachedPixels()
{
    if (pixels) {
        stbi_image_free(pixels);
        pixels = nullptr;
    }
}

void ImageInternal::updatePixels()
{
    if (!pixels) {
        throw logic_error("Cannot updatePixels unless image pixels are cached!");
    }
    nvgUpdateImage(vg, vgImageIdx, reinterpret_cast<const unsigned char*>(pixels));
}

ModKey cvtMods(int glfwMods)
{
    ModKey mods = static_cast<ModKey>(
                ((glfwMods & GLFW_MOD_CONTROL) ? (int)ModKey::Ctrl  : 0) |
                ((glfwMods & GLFW_MOD_ALT)     ? (int)ModKey::Alt   : 0) |
                ((glfwMods & GLFW_MOD_SHIFT)   ? (int)ModKey::Shift : 0));

    return mods;
}


void key(GLFWwindow* window, int key, int /*scancode*/, int action, int mods)
{
    Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));

    switch (key) {
    case GLFW_KEY_RIGHT:
        key = static_cast<int>(Key::Right);
        break;
    case GLFW_KEY_LEFT:
        key = static_cast<int>(Key::Left);
        break;
    case GLFW_KEY_UP:
        key = static_cast<int>(Key::Up);
        break;
    case GLFW_KEY_DOWN:
        key = static_cast<int>(Key::Down);
        break;
    }

    if (key == GLFW_KEY_ENTER &&
            mods & GLFW_MOD_ALT &&
            action == GLFW_PRESS) {
        g->toggleFullScreen();
    }

    Vec2d mp = g->mousePos();

    switch (action) {
    case GLFW_PRESS:
        g->postEvent(mp.x, mp.y, EvtType::KeyPress, cvtMods(mods), key);
        break;
    case GLFW_RELEASE:
        g->postEvent(mp.x, mp.y, EvtType::KeyRelease, cvtMods(mods), key);
        break;
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));

    Vec2d mp = g->mousePos();

    switch (button) {
    case 0:
        button = 1;
        break;
    case 1:
        button = 2;
        break;
    case 2:
        button = 4;
        break;
    default:
        button = -1;
    }

    switch (action) {
    case GLFW_PRESS:
        //cout << "Click: " << mp.x << " " << mp.y << endl;
        g->postEvent(mp.x, mp.y,EvtType::MousePress, cvtMods(mods), button);
        break;
    case GLFW_RELEASE:
        g->postEvent(mp.x, mp.y,EvtType::MouseRelease, cvtMods(mods), button);
        break;
    }
}

void mousePosCallback(GLFWwindow* window, double x, double y)
{
    Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));

    g->setMousePos(x,y);

    int buttons =
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ? 1 : 0) +
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ? 2 : 0) +
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) ? 4 : 0);

    if (buttons > 0)
    {
        g->postEvent(x, y, EvtType::MouseMove, cvtMods(0), buttons);
    }
}

void scrollWheelCallback(GLFWwindow* window, double /*sx*/, double sy)
{
    Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //cout << "Scroll: " << sy << endl;
    Vec2d mp = g->mousePos();

    g->postEvent(mp.x, mp.y, EvtType::MouseWheel, cvtMods(0), sy < 0 ? -1 : 1);
}

#ifndef _MSC_VER
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

void windowPosCallback(GLFWwindow* window,int x,int y)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowPosCallback " << x << " " << y << endl;
}

void windowSizeCallback(GLFWwindow* window,int width,int height)
{
    //cout << "windowSizeCallback: " << width << " " << height << endl;
    Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    g->gotResizeEvent = true;
}

void windowCloseCallback(GLFWwindow* window)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowCloseCallback" << endl;
}

void windowRefreshCallback(GLFWwindow* window)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowRefreshCallback" << endl;
}


void windowFocusCallback(GLFWwindow* window, int hasFocus)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowFocusCallback " << hasFocus << endl;
}

void windowIconifyCallback(GLFWwindow* window, int isIconified)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowIconifyCallback " << isIconified << endl;
}

void windowMaximizeCallback(GLFWwindow* window, int maxArg)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowMaximizeCallback " << maxArg << endl;
}

void frameBufferSizeCallback(GLFWwindow* window, int width, int height)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "frameBufferSizeCallback " << width << " " << height << endl;
}

void windowContentScaleCallback(GLFWwindow* window, float xArg, float yArg)
{
    //Graphics* g = reinterpret_cast<Graphics*>(glfwGetWindowUserPointer(window));
    //::cout << "windowContentScaleCallback " << xArg << " " << yArg << endl;
}





} // namespace mssm

void errorcb(int error, const char* desc)
{
    printf("GLFW error %d: %s\n", error, desc);
}




#ifndef _MSC_VER
#pragma GCC diagnostic pop
#endif


Graphics::Graphics(std::string title, int width, int height,
                   std::function<void (Graphics&)> mainFunc)

{   
#ifdef INCLUDE_SOUND
    soundPlayer.init();
#endif


    this->mainFunc = mainFunc;

    currentWidth = width;
    currentHeight = height;

    this->title = title;
    background = BLACK;

    start_time =
            std::chrono::duration_cast<std::chrono::microseconds>
            (std::chrono::steady_clock::now().time_since_epoch()).count();
    lastDrawTime = std::chrono::steady_clock::now();

    isPressed.resize(GLFW_KEY_LAST+1);
    //wasPressed.resize(GLFW_KEY_LAST+1);


    //    bool supportTear = (glfwExtensionSupported("WGL_EXT_swap_control_tear") ||
    //                 glfwExtensionSupported("GLX_EXT_swap_control_tear"));

    glfwSetErrorCallback(errorcb);

    if (!glfwInit()) {
        printf("Failed to init GLFW.");
        return;
    }




#ifndef _WIN32 // don't require this on win32, and works with more cards
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_FALSE );

#ifdef DEMO_MSAA
    glfwWindowHint(GLFW_SAMPLES, 4);
#endif
    //    window =   glfwCreateWindow(glfwGetVideoMode(glfwGetPrimaryMonitor())->width,
    //                                               glfwGetVideoMode(glfwGetPrimaryMonitor())->height, "My Title",
    //                                               glfwGetPrimaryMonitor(), nullptr);

    window = glfwCreateWindow(currentWidth, currentHeight, title.c_str(), NULL, NULL);
    //	window = glfwCreateWindow(1000, 600, "NanoVG", glfwGetPrimaryMonitor(), NULL);
    if (!window) {
        glfwTerminate();
        return;
    }

#ifdef _WIN32
    hdc = GetDC(glfwGetWin32Window(window));
#endif

    //glfwSetWindowIcon(window, count, icons);

    glfwSetWindowUserPointer(window, this);

    glfwSetKeyCallback(window, key);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, mousePosCallback);
    glfwSetScrollCallback(window, scrollWheelCallback);

    glfwSetWindowPosCallback (window, windowPosCallback);
    glfwSetWindowSizeCallback (window, windowSizeCallback);
    glfwSetWindowCloseCallback (window, windowCloseCallback);
    glfwSetWindowRefreshCallback (window, windowRefreshCallback);
    glfwSetWindowFocusCallback (window, windowFocusCallback);
    glfwSetWindowIconifyCallback (window, windowIconifyCallback);
    glfwSetWindowMaximizeCallback (window, windowMaximizeCallback);
    glfwSetFramebufferSizeCallback (window, frameBufferSizeCallback);
    glfwSetWindowContentScaleCallback (window, windowContentScaleCallback);

    glfwMakeContextCurrent(window);
#ifdef NANOVG_GLEW
    glewExperimental = GL_TRUE;
    if(glewInit() != GLEW_OK) {
        printf("Could not init glew.\n");
        return;
    }
    // GLEW generates GL error because it calls glGetString(GL_EXTENSIONS), we'll consume it here.
    glGetError();
#endif

#ifdef DEMO_MSAA
    vg = nvgCreateGL3(NVG_STENCIL_STROKES | NVG_DEBUG);
#else
    vg = nvgCreateGL3(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
#endif
    if (vg == NULL) {
        printf("Could not init nanovg.\n");
        return;
    }

    //    if (loadDemoData2(vg, &data) == -1)
    //        return -1;

    glfwSwapInterval(1); // supportTear ? -1 : 1);

#ifdef _WIN32
    // Turn on vertical screen sync under Windows.
    // (I.e. it uses the WGL_EXT_swap_control extension)
    typedef BOOL (WINAPI *PFNWGLSWAPINTERVALEXTPROC)(int interval);
    PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT = NULL;
#ifndef _MSC_VER
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
    wglSwapIntervalEXT = reinterpret_cast<PFNWGLSWAPINTERVALEXTPROC>(wglGetProcAddress("wglSwapIntervalEXT"));
#ifndef _MSC_VER
#pragma GCC diagnostic pop
#endif
    if(wglSwapIntervalEXT) {
        wglSwapIntervalEXT(1);
    }
#endif

    //	initGPUTimer(&gpuTimer);

    glfwSetTime(0);

    fontRegular = nvgCreateFont(vg, "sans", Paths::findAsset("Roboto-Regular.ttf").c_str());
    if (fontRegular == -1) {
        printf("Could not add font italic.\n");
    }
    fontBold = nvgCreateFont(vg, "sans-bold", Paths::findAsset("Roboto-Bold.ttf").c_str());
    if (fontBold == -1) {
        printf("Could not add font bold.\n");
    }
    fontLight = nvgCreateFont(vg, "sans-light", Paths::findAsset("Roboto-Light.ttf").c_str());
    if (fontLight == -1) {
        printf("Could not add font light.\n");
    }

    draw();

    if (mainFunc) {
        mainFunc(*this);
    }
}

Graphics::~Graphics()
{   
    keepImages.clear();
    nvgDeleteGL3(vg);
    glfwTerminate();
    setClosed();
#ifdef INCLUDE_SOUND
    soundPlayer.deinit();
#endif
}


bool Graphics::isKeyPressed(Key k)
{
    return isKeyPressed((int)k);
}


std::ostream& mssm::operator<<(std::ostream& os, const Event& evt)
{
    os << "Evt: ";

    switch (evt.evtType) {
    case EvtType::KeyPress:     os << "KeyPress    "; break;
    case EvtType::KeyRelease:   os << "KeyRelease  "; break;
    case EvtType::MouseMove:    os << "MouseMove   "; break;
    case EvtType::MousePress:   os << "MousePress  "; break;
    case EvtType::MouseRelease: os << "MouseRelease"; break;
    case EvtType::MouseWheel:   os << "MouseWheel  "; break;
    case EvtType::MusicEvent:   os << "MusicEvent  "; break;
    case EvtType::WindowResize: os << "WindowResize"; break;
    }

    os << " x: " << setw(5) << evt.x << " y: " << setw(5) << evt.y << " arg: " << evt.arg;

    if (static_cast<int>(evt.mods) & static_cast<int>(ModKey::Ctrl))  { os << " <CTRL>";  }
    if (static_cast<int>(evt.mods) & static_cast<int>(ModKey::Alt))   { os << " <ALT>";   }
    if (static_cast<int>(evt.mods) & static_cast<int>(ModKey::Shift)) { os << " <SHIFT>"; }

    //os << " data: " << evt.data << " pluginId: " << evt.pluginId;

    return os;
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
        if (arg <= GLFW_KEY_LAST)
        {
            isPressed[arg]  = true;
        }
        break;
    case EvtType::KeyRelease:
        if (arg <= GLFW_KEY_LAST)
        {
            isPressed[arg] = false;
        }
        break;
    default:
        break;
    }

    _events.emplace_back(std::move(evt));
}

Vec2d Graphics::mousePos()
{
    return _mousePos;
//    Vec2d pos;
//    glfwGetCursorPos(window, &pos.x, &pos.y);
//    return pos;
}


void Graphics::setClosed()
{


    closed  = true;

}

bool Graphics::isClosed()
{
    return closed;
}



void Graphics::setMousePos(int x, int y)
{
    _mousePos.x = x;
    _mousePos.y = y;
}

std::chrono::milliseconds::rep Graphics::time()
{
    auto milliseconds_since_epoch =
            std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now().time_since_epoch()).count();

    return milliseconds_since_epoch - start_time;
}

void Graphics::test()
{
    Graphics &g = *this; // this is so the code below can
    // be easily copy/pasted into main

    double a = 0;

    vector<double> times;

    Image skull(g, "skull.png");
    Image pattern(g, "pattern.png");
    Sound pew(g, "ShortLaser.wav");

    while (g.draw()) {

        a += g.elapsedMs()/10*M_PI/180.0;

        Vec2d center{g.width()/2, g.height()/2};

        Vec2d corner = center + Vec2d{100, 0}.rotated(a);

        g.line(center, corner);

        times.push_back(g.elapsedMs());
        if (times.size() > 200) {
            times.erase(times.begin());
        }

        double timeSum = 0;
        for (int i = 0; i < times.size(); i++) {
            g.line({0, i+50}, {times[i]*10, i+50}, GREEN);
            timeSum += times[i];
        }

        double aveFps = 1000.0 / (timeSum/times.size());

        g.rect(corner, 50, 100, CYAN, BLUE);

        double fps = 1000.0 / g.elapsedMs();

        g.text(corner, 20, "Average fps: " + to_string(aveFps), WHITE);
        g.text({20,200}, 20, "fps: " + to_string(fps), WHITE);

        g.ellipse({0,0}, 50, 50, YELLOW, YELLOW);
        g.ellipse({0,0}, 40, 40, BLACK, BLACK);

        g.image({300,200}, pattern);

        g.imageC(g.mousePos(), a, skull);

        g.line({0,0}, g.mousePos(), GREEN);

        if (g.isKeyPressed(Key::ESC)) {
            break;
        }

        for (const Event& e : g.events()) {
            g.cerr << e << endl;

            switch (e.evtType) {
            case EvtType::KeyPress:
                if (e.arg == ' ') {
                    g.play(pew);
                }
                break;
            default:
                break;
            }
        }
    }

}

void Graphics::waitUntilClosed()
{
    while (!glfwWindowShouldClose(window)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        glfwPollEvents();
    }
    setClosed();
}

std::vector<Event> Graphics::events()
{
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

bool Graphics::appendOutputText(const std::string& txt)
{
    stringOutput.append(txt);
    return true;
}


void Graphics::music(const std::string& filename)
{
    //qDebug() << "graphicsMain " << QThread::currentThreadId() << endl;



    musicFile = Paths::findAsset(filename);
}

void Graphics::play(Sound sound)
{
#ifdef INCLUDE_SOUND
    sound.sound->play();
#endif
}

void splitRows(std::deque<std::string>& lines, const std::string& str, size_t keepLast)
{
    const char* ptr = str.c_str();
    const char* eptr = ptr + str.size();
    const char* sptr = ptr;
    while (ptr < eptr) {
        if (*ptr == 10) {
            lines.emplace_back(sptr, ptr-sptr);
            if (keepLast > 0 && lines.size() > keepLast) {
                lines.erase(lines.begin(), lines.begin() + lines.size() - keepLast);
            }
            ptr++;
            if (ptr < eptr && *ptr == 13) {
                ptr++;
            }
            sptr = ptr;
        }
        else if (*ptr == 13) {
            lines.emplace_back(sptr, ptr-sptr);
            if (keepLast > 0 && lines.size() > keepLast) {
                lines.erase(lines.begin(), lines.begin() + lines.size() - keepLast);
            }
            ptr++;
            if (ptr < eptr && *ptr == 10) {
                ptr++;
            }
            sptr = ptr;
        }
        else {
            ptr++;
        }
    }
    if (ptr > sptr) {
        lines.emplace_back(sptr, ptr-sptr);
        if (keepLast > 0 && lines.size() > keepLast) {
            lines.erase(lines.begin(), lines.begin() + lines.size() - keepLast);
        }
    }
}

bool Graphics::draw()
{
    if (closed || glfwWindowShouldClose(window)) {
        setClosed();
        return false;
    }

    if (requestToggleFullScreen) {
        requestToggleFullScreen = false;

        if (glfwGetWindowMonitor(window)) {
            // it was full screen (otherwise glfwGetWindowMonitor would have returned null)
            // switch to windowed mode
            //::cout << "Window Mode: " << windowedX << " " << windowedY << " " << currentWidth << " " << currentHeight << endl;
            glfwSetWindowMonitor(window, NULL, windowedX, windowedX, currentWidth, currentHeight, 0);
        }
        else
        {
            // we were windowed and are switching to full screen
            GLFWmonitor* monitor = get_current_monitor(window);  // get the monitor the window is most on

            // save the windowed confuration for later if we go out of full screen
            glfwGetWindowPos(window, &windowedX, &windowedY);
            glfwGetWindowSize(window, &windowedWidth, &windowedHeight);

            //::cout << "Leaving windowed Mode: " << windowedX << " " << windowedY << " " << windowedWidth << " " << windowedHeight << endl;

            const GLFWvidmode* mode = glfwGetVideoMode(monitor);

            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);

            //::cout << "FullScreen Mode: " << mode->width << " " << mode->height << endl;
            //::cout << "Refresh: " << mode->refreshRate << endl;
        }
    }

    std::deque<std::string> lines;
    drawFromStream(cout, lines, { 10, 20 }, WHITE);
    drawFromStream(cerr, cerrLines, { currentWidth/2, 20 }, RED);

    nvgEndFrame(vg);

    keepImages.clear();

    //    if (WGLEW_NV_delay_before_swap) {
    //       wglDelayBeforeSwapNV(hdc, 0.01);
    //    }

    glfwSwapBuffers(window);

    glfwPollEvents();

    int winWidth, winHeight;
    glfwGetWindowSize(window, &winWidth, &winHeight);
    currentWidth = winWidth;
    currentHeight = winHeight;

    if (gotResizeEvent) {
        gotResizeEvent = false;
        postEvent(currentWidth, currentHeight, EvtType::WindowResize, cvtMods(0), 0);
    }

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    // Calculate pixel ration for hi-dpi devices.
    float pxRatio = (float)fbWidth / (float)winWidth;

    // Update and render
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    nvgBeginFrame(vg, winWidth, winHeight, pxRatio);

    std::chrono::steady_clock::time_point currDrawTime = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(currDrawTime - lastDrawTime).count();
    lastDrawTime = currDrawTime;

    return true;
}

void Graphics::drawFromStream(std::stringstream &ss, std::deque<std::string>& lines, Vec2d start, Color c)
{
    string tmp = ss.str();
    if (!tmp.empty()) {
        splitRows(lines, tmp, (currentHeight-40)/25);
        ss.str("");
    }

    if (!lines.empty()) {
        nvgFontSize(vg, 20.0f);
        nvgFillColor(vg, nvgRGBA(c.r, c.g, c.b, 255));
        nvgTextAlign(vg, NVG_ALIGN_LEFT|NVG_ALIGN_TOP);
    }

    double y = start.y;
    for (const auto& row : lines) {
        const char* ptr = row.c_str();
        nvgText(vg, start.x, y, ptr, ptr + row.size());
        y += 25;
    }
}


void Graphics::clear()
{
    cleared = true;
}


void Graphics::snapShot(Image &image)
{


    image.set(currentWidth, currentHeight, background);
    throw new logic_error("Not Implemented snapshot");
    //    QPainter painter(image.pixmap.get());

    //    for (auto& g : active)
    //    {
    //        g->draw(&painter);
    //    }
}

void Graphics::image(Vec2d pos, const Image& img)
{
    keepImages.push_back(img.pixmap);

    const float w = img.width();
    const float h = img.height();
    NVGpaint ip = nvgImagePattern(vg, pos.x, pos.y, w, h, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgBeginPath(vg);
    nvgRect(vg, pos.x, pos.y, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::image(Vec2d pos, const Image& img, Vec2d src, int srcw, int srch)
{
    keepImages.push_back(img.pixmap);

    const float w = img.width();
    const float h = img.height();
    //   _checkAlignPixelsAdjust(&dx, &dy);
    //   _checkAlignPixels(&dw, &dh);
    const float ratiox = w/srcw;
    const float ratioy = h/srch;
    float startX = pos.x - src.x * ratiox;
    float startY = pos.y - src.y * ratioy;
    float pw = img.width()  * ratiox;
    float ph = img.height() * ratioy;
    NVGpaint ip = nvgImagePattern(vg, startX, startY, pw, ph, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgBeginPath(vg);
    nvgRect(vg, pos.x, pos.y, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::image(Vec2d pos, double w, double h, const Image& img)
{
    keepImages.push_back(img.pixmap);

    if (w < 0) {
        pos.x -= w;
    }
    if (h < 0) {
        pos.y -= h;
    }
    //   _checkAlignPixelsAdjust(&dx, &dy);
    //   _checkAlignPixels(&dw, &dh);
    NVGpaint ip = nvgImagePattern(vg, pos.x, pos.y, w, h, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgBeginPath(vg);
    nvgRect(vg, pos.x, pos.y, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::image(Vec2d pos, double w, double h, const Image& img, Vec2d src, int srcw, int srch)
{
    keepImages.push_back(img.pixmap);

    if (w < 0) {
        pos.x -= w;
    }
    if (h < 0) {
        pos.y -= h;
    }
    //   _checkAlignPixelsAdjust(&dx, &dy);
    //   _checkAlignPixels(&dw, &dh);
    const float ratiox = w/srcw;
    const float ratioy = h/srch;
    float startX = pos.x - src.x * ratiox;
    float startY = pos.y - src.y * ratioy;
    float pw = img.width()  * ratiox;
    float ph = img.height() * ratioy;
    NVGpaint ip = nvgImagePattern(vg, startX, startY, pw, ph, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgBeginPath(vg);
    nvgRect(vg, pos.x, pos.y, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::imageC(Vec2d center, double angle, const Image &img)
{
    keepImages.push_back(img.pixmap);

    const float w = img.width();
    const float h = img.height();
    const float offx = -w/2;
    const float offy = -h/2;
    NVGpaint ip = nvgImagePattern(vg, offx, offy, w, h, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgTranslate(vg, center.x, center.y);
    if (angle != 0) {
        nvgRotate(vg, angle);
    }
    nvgBeginPath(vg);
    nvgRect(vg, offx, offy, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::imageC(Vec2d center, double angle, const Image &img, Vec2d src, int srcw, int srch)
{
    keepImages.push_back(img.pixmap);

    //   _checkAlignPixelsAdjust(&dx, &dy);
    //   _checkAlignPixels(&dw, &dh);
    const float offx = -srcw/2;
    const float offy = -srch/2;
    const float ratiox = srcw/srcw;
    const float ratioy = srch/srch;
    float startX = offx - src.x * ratiox;
    float startY = offy - src.y * ratioy;
    float pw = img.width()  * ratiox;
    float ph = img.height() * ratioy;
    NVGpaint ip = nvgImagePattern(vg, startX, startY, pw, ph, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgTranslate(vg, center.x, center.y);
    if (angle != 0) {
        nvgRotate(vg, angle);
    }
    nvgBeginPath(vg);
    nvgRect(vg, offx, offy, srcw, srch);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::imageC(Vec2d center, double angle, double w, double h, const Image &img)
{
    keepImages.push_back(img.pixmap);

    const float offx = -w/2;
    const float offy = -h/2;
    NVGpaint ip = nvgImagePattern(vg, offx, offy, w, h, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgTranslate(vg, center.x, center.y);
    if (angle != 0) {
        nvgRotate(vg, angle);
    }
    nvgBeginPath(vg);
    nvgRect(vg, offx, offy, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::imageC(Vec2d center, double angle, double w, double h, const Image &img, Vec2d src, int srcw, int srch)
{
    keepImages.push_back(img.pixmap);

    //   _checkAlignPixelsAdjust(&dx, &dy);
    //   _checkAlignPixels(&dw, &dh);
    const float offx = -w/2;
    const float offy = -h/2;
    const float ratiox = w/srcw;
    const float ratioy = h/srch;
    float startX = offx - src.x * ratiox;
    float startY = offy - src.y * ratioy;
    float pw = img.width()  * ratiox;
    float ph = img.height() * ratioy;
    NVGpaint ip = nvgImagePattern(vg, startX, startY, pw, ph, 0.0f, img.pixmap->vgImageIdx, 1.0f);
    nvgSave(vg);
    nvgTranslate(vg, center.x, center.y);
    if (angle != 0) {
        nvgRotate(vg, angle);
    }
    nvgBeginPath(vg);
    nvgRect(vg, offx, offy, w, h);
    nvgFillPaint(vg, ip);
    nvgFill(vg);
    nvgRestore(vg);
}

void Graphics::line(Vec2d p1, Vec2d p2, Color c)
{
    nvgBeginPath(vg);
    nvgMoveTo(vg, p1.x, p1.y);
    nvgLineTo(vg, p2.x, p2.y);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    nvgStroke(vg);
}

void Graphics::ellipse(Vec2d pos, double w, double h, Color c, Color fill)
{
    nvgBeginPath(vg);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    nvgFillColor(vg, nvgRGBA(fill.r, fill.g, fill.b, fill.a));
    nvgEllipse(vg, pos.x, pos.y, w/2.0, h/2.0);
    nvgFill(vg);
    nvgStroke(vg);
}
//d nvgArc2(NVGcontext* ctx, float cx, float cy, float rx, float ry, float a0, float a1, int dir);

void Graphics::arc(Vec2d pos, double w, double h, double a, double alen, Color c)
{
    nvgBeginPath(vg);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    nvgArc2(vg, pos.x, pos.y, w/2.0, h/2.0, -a, -a-alen, NVG_CCW);
    nvgStroke(vg);
}

void Graphics::chord(Vec2d pos, double w, double h, double a, double alen, Color c, Color fill)
{
    nvgBeginPath(vg);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    nvgFillColor(vg, nvgRGBA(fill.r, fill.g, fill.b, fill.a));
    nvgArc2(vg, pos.x, pos.y, w/2.0, h/2.0, -a, -a-alen, NVG_CCW);
    nvgClosePath(vg);
    nvgFill(vg);
    nvgStroke(vg);
}

void Graphics::pie(Vec2d pos, double w, double h, double a, double alen, Color c, Color fill)
{
    nvgBeginPath(vg);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    nvgFillColor(vg, nvgRGBA(fill.r, fill.g, fill.b, fill.a));
    nvgMoveTo(vg, pos.x,pos.y);
    nvgArc2(vg, pos.x, pos.y, w/2.0, h/2.0, -a, -a-alen, NVG_CCW);
    nvgLineTo(vg, pos.x,pos.y);
    nvgFill(vg);
    nvgStroke(vg);
}

void Graphics::rect(Vec2d corner, double w, double h, Color c, Color fill)
{
    nvgBeginPath(vg);
    nvgRect(vg, corner.x, corner.y, w, h);
    if (fill.a > 0) {
        nvgFillColor(vg, nvgRGBA(fill.r, fill.g, fill.b, fill.a));
        nvgFill(vg);
    }
    if (c.a > 0) {
        nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
        nvgStroke(vg);
    }
}

void Graphics::polygon(std::vector<Vec2d> points, Color border, Color fill)
{
    if (points.empty()) {
        return;
    }

    nvgBeginPath(vg);
    nvgMoveTo(vg, points[0].x, points[0].y);

    for (size_t i = 1; i < points.size(); i++) {
        nvgLineTo(vg, points[i].x, points[i].y);
    }

    nvgClosePath(vg);

    if (fill.a > 0) {
        if (fill.a > 0) {
            nvgFillColor(vg, nvgRGBA(fill.r, fill.g, fill.b, fill.a));
            nvgFill(vg);
        }
    }

    if (border.a > 0) {
        nvgStrokeColor(vg, nvgRGBA(border.r, border.g, border.b, border.a));
        nvgStroke(vg);
    }
}

void Graphics::polyline(std::vector<Vec2d> points, Color color)
{
    nvgBeginPath(vg);
    nvgMoveTo(vg, points[0].x, points[0].y);
    for (size_t i = 1; i < points.size(); i++) {
        nvgLineTo(vg, points[i].x, points[i].y);
    }
    if (color.a > 0) {
        nvgStrokeColor(vg, nvgRGBA(color.r, color.g, color.b, color.a));
        nvgStroke(vg);
    }
}

void Graphics::text(Vec2d pos, double size, const std::string &str, Color c, HAlign hAlign, VAlign vAlign)
{
    nvgFontSize(vg, size);
    //nvgFontFace(vg, "sans");
    nvgFontFaceId(vg, fontRegular);
    nvgTextAlign(vg, static_cast<int>(hAlign) | static_cast<int>(vAlign));
    // nvgFontBlur(vg,2);
    nvgFillColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    auto s = str.c_str();
    nvgText(vg, pos.x, pos.y, s, s+str.length());
}

void Graphics::textExtents(double size, const string &str, TextExtents &extents)
{
    float metrics[4];

    nvgFontSize(vg, size);
    //nvgFontFace(vg, "sans");
    nvgFontFaceId(vg, fontRegular);
    auto s = str.c_str();
    float advance = nvgTextBounds(vg, 0, 0, s, s+str.length(), metrics);

    extents.textHeight = metrics[3]-metrics[1];  // ymax-ymin
    extents.textWidth  = metrics[2]-metrics[0];  // xmax-xmin

    extents.textAdvance = advance;
    nvgTextMetrics(vg, &extents.fontAscent, &extents.fontDescent, &extents.fontHeight);
}

void Graphics::point(Vec2d p, Color c)
{
    nvgBeginPath(vg);
    nvgStrokeWidth(vg, 3);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    nvgMoveTo(vg, p.x-1, p.y-1);
    nvgLineTo(vg, p.x+1, p.y+1);
    nvgStroke(vg);
    nvgStrokeWidth(vg, 1);
}

void Graphics::points(std::vector<Vec2d> pts, Color c)
{
    nvgBeginPath(vg);
    nvgStrokeWidth(vg, 3);
    nvgStrokeColor(vg, nvgRGBA(c.r, c.g, c.b, c.a));
    for (auto& p : pts) {
        nvgMoveTo(vg, p.x-1,p.y-1);
        nvgLineTo(vg, p.x+1,p.y+1);
    }
    nvgStroke(vg);
    nvgStrokeWidth(vg, 1);
}

double Graphics::timeMs()
{
    auto microseconds_since_epoch =
            std::chrono::duration_cast<std::chrono::microseconds>
            (std::chrono::steady_clock::now().time_since_epoch()).count();

    return microseconds_since_epoch - start_time;
}


//Widget::Widget(mssm::Graphics *graphics, Window *parent)
//    : QNanoWidget(parent), _graphics(graphics), _parent(parent)
//{
//    graphics->updator = [this]() { update(); };

//    lastTime = graphics->timeMs();

//    setMinimumHeight(graphics->height());
//    setMinimumWidth(graphics->width());

//    setFocusPolicy(Qt::StrongFocus);
//    setFocus();
//}

//void Widget::paint(QNanoPainter *p)
//{
//    auto newTime = _graphics->timeMs();
//    auto elapsed = newTime - lastTime;
//    lastTime = newTime;

//    _graphics->draw(this, p, width(), height(), elapsed);

//    auto cursorPos = mapFromGlobal(QCursor::pos());

//    _graphics->setMousePos(cursorPos.x(), cursorPos.y());
//}

//void Widget::animate()
//{
//    // update();
//}

//void Widget::paintEvent(QPaintEvent *event)
//{
//    QNanoWidget::paintEvent(event);



//}




//ModKey cvtMods(Qt::KeyboardModifiers qmods)
//{
//    ModKey mods = static_cast<ModKey>(
//                ((qmods & Qt::ControlModifier) ? (int)ModKey::Ctrl : 0) |
//                ((qmods & Qt::AltModifier) ? (int)ModKey::Alt : 0) |
//                ((qmods & Qt::ShiftModifier) ? (int)ModKey::Shift : 0));

//    return mods;
//}



//void Widget::mousePressEvent(QMouseEvent * event)
//{
//    _graphics->postEvent(event->x(), event->y(), EvtType::MousePress,
//                           cvtMods(event->modifiers()),
//                           (int)event->button());
//}

//void Widget::mouseReleaseEvent(QMouseEvent * event)
//{
//    _graphics->postEvent(event->x(), event->y(), EvtType::MouseRelease,
//                           cvtMods(event->modifiers()),
//                           (int)event->button());
//}

//void Widget::mouseMoveEvent(QMouseEvent * event)
//{
//    _graphics->postEvent(event->x(), event->y(), EvtType::MouseMove,
//                           cvtMods(event->modifiers()),
//                           (int)event->buttons());
//}

//void Widget::keyPressEvent(QKeyEvent * event)
//{
//    _graphics->postEvent(0, 0, EvtType::KeyPress,
//                           cvtMods(event->modifiers()),
//                           event->key());
//}

//void Widget::keyReleaseEvent(QKeyEvent * event)
//{
//    _graphics->postEvent(0,0, EvtType::KeyRelease,
//                           cvtMods(event->modifiers()),
//                           event->key());
//}

//Window::Window(mssm::Graphics *g, std::string title)
//{
//    graphics = g;

//    g->setInputTextFunc([this]() { return getInputText(); });

//    setWindowTitle(tr(title.c_str()));

//    graphicsWidget = new Widget(graphics, this);
//    streamOutBox = new QPlainTextEdit;
//    streamInBox = new QLineEdit;

//    QGridLayout *layout = new QGridLayout;

//    layout->setRowStretch(0,1);
//    layout->setRowStretch(1,0);
//    layout->setRowStretch(2,0);

//    layout->addWidget(graphicsWidget, 0, 0);
//    layout->addWidget(streamOutBox, 1, 0);
//    layout->addWidget(streamInBox, 2, 0);

//    streamOutBox->setMaximumBlockCount(100);

//    streamInBox->hide();
//    streamOutBox->hide();
//    streamInBox->setEnabled(false);

//    connect(streamInBox, SIGNAL(returnPressed()), this, SLOT(textEntered()));

//    setLayout(layout);


//}

//void Window::appendOutputText(const std::string& txt)
//{
//    if (graphics->isClosed())
//    {
//        return;
//    }

//    if (streamOutBox->isHidden())
//    {
//        streamOutBox->show();
//    }

//    streamOutBox->moveCursor(QTextCursor::End);
//    streamOutBox->insertPlainText(QString::fromStdString(txt));
//    streamOutBox->moveCursor(QTextCursor::End);
//}

//std::string Window::getInputText()
//{
//    if (graphics->isClosed())
//    {
//        return "";
//    }

//    if (!usingStreamIO)
//    {
//        usingStreamIO = true;
//        QMetaObject::invokeMethod( streamInBox, "show");
//    }

//    QMetaObject::invokeMethod( streamInBox, "setEnabled", Q_ARG(bool, true));
//    QMetaObject::invokeMethod( streamInBox, "setFocus");

//    std::unique_lock<std::mutex> lock(wlock);

//    cv.wait(lock, [this]{ return hasEnteredText; });

//    string str;

//    if (hasEnteredText)
//    {
//        std::swap(str, enteredText);
//        hasEnteredText = false;
//    }

//    return str;
//}

//void Window::textEntered()
//{
//    auto str = streamInBox->text().toStdString();

//    streamInBox->clear();

//    std::unique_lock<std::mutex> lock(wlock);

//    enteredText.append(str);
//    enteredText.append("\n");

//    hasEnteredText = true;

//    streamInBox->setEnabled(false);
//    graphicsWidget->setFocus();

//    cv.notify_all();
//}

//void Window::musicStateChanged(QMediaPlayer::State state)
//{
//    qDebug() << "Music State Changed: " << static_cast<int>(state) << " Thread: " << QThread::currentThreadId();

//    const char *msg = "";
//    int stateNum = 0;

//    switch (state) {
//    case QMediaPlayer::PausedState:
//        msg = "MusicPaused";
//        stateNum = 2;
//        break;
//    case QMediaPlayer::PlayingState:
//        msg = "MusicPlaying";
//        stateNum = 1;
//        break;
//    case QMediaPlayer::StoppedState:
//        msg = "MusicStopped";
//        stateNum = 0;
//        break;
//    }

//    graphics->postEvent(0, 0, EvtType::MusicEvent, ModKey{}, stateNum, 0, msg);
//}

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
    int    i{static_cast<int>(hh)};
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

// h:0-360.0, s:0.0-1.0, v:0.0-1.0
void rgb2hsv(Color c, double &h, double &s, double &v)
{
    double r = c.r / 255.0;
    double g = c.g / 255.0;
    double b = c.b / 255.0;

    double mx = max(r, max(g, b));
    double mn = min(r, min(g, b));

    v = mx;

    if (mx == 0.0f) {
        s = 0;
        h = 0;
    }
    else if (mx - mn == 0.0f) {
        s = 0;
        h = 0;
    }
    else {
        s = (mx - mn) / mx;

        if (mx == r) {
            h = 60 * ((g - b) / (mx - mn)) + 0;
        }
        else if (mx == g) {
            h = 60 * ((b - r) / (mx - mn)) + 120;
        }
        else {
            h = 60 * ((r - g) / (mx - mn)) + 240;
        }
    }

    if (h < 0) h += 360.0f;
}


//typedef struct {
//    double r;       // a fraction between 0 and 1
//    double g;       // a fraction between 0 and 1
//    double b;       // a fraction between 0 and 1
//} rgb;

//typedef struct {
//    double h;       // angle in degrees
//    double s;       // a fraction between 0 and 1
//    double v;       // a fraction between 0 and 1
//} hsv;

//static hsv   rgb2hsv(rgb in);
//static rgb   hsv2rgb(hsv in);

//hsv rgb2hsv(rgb in)
//{
//    hsv         out;
//    double      min, max, delta;

//    min = in.r < in.g ? in.r : in.g;
//    min = min  < in.b ? min  : in.b;

//    max = in.r > in.g ? in.r : in.g;
//    max = max  > in.b ? max  : in.b;

//    out.v = max;                                // v
//    delta = max - min;
//    if (delta < 0.00001)
//    {
//        out.s = 0;
//        out.h = 0; // undefined, maybe nan?
//        return out;
//    }
//    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
//        out.s = (delta / max);                  // s
//    } else {
//        // if max is 0, then r = g = b = 0
//        // s = 0, h is undefined
//        out.s = 0.0;
//        out.h = NAN;                            // its now undefined
//        return out;
//    }
//    if( in.r >= max )                           // > is bogus, just keeps compilor happy
//        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
//    else
//    if( in.g >= max )
//        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
//    else
//        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

//    out.h *= 60.0;                              // degrees

//    if( out.h < 0.0 )
//        out.h += 360.0;

//    return out;
//}


//rgb hsv2rgb(hsv in)
//{
//    double      hh, p, q, t, ff;
//    long        i;
//    rgb         out;

//    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
//        out.r = in.v;
//        out.g = in.v;
//        out.b = in.v;
//        return out;
//    }
//    hh = in.h;
//    if(hh >= 360.0) hh = 0.0;
//    hh /= 60.0;
//    i = (long)hh;
//    ff = hh - i;
//    p = in.v * (1.0 - in.s);
//    q = in.v * (1.0 - (in.s * ff));
//    t = in.v * (1.0 - (in.s * (1.0 - ff)));

//    switch(i) {
//    case 0:
//        out.r = in.v;
//        out.g = t;
//        out.b = p;
//        break;
//    case 1:
//        out.r = q;
//        out.g = in.v;
//        out.b = p;
//        break;
//    case 2:
//        out.r = p;
//        out.g = in.v;
//        out.b = t;
//        break;

//    case 3:
//        out.r = p;
//        out.g = q;
//        out.b = in.v;
//        break;
//    case 4:
//        out.r = t;
//        out.g = p;
//        out.b = in.v;
//        break;
//    case 5:
//    default:
//        out.r = in.v;
//        out.g = p;
//        out.b = q;
//        break;
//    }
//    return out;
//}



} // namespace mssm

#ifdef PROVIDE_WINMAIN
int main();

int CALLBACK WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
   return main();
}
#endif


