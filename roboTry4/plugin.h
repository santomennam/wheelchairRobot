#ifndef PLUGIN_H
#define PLUGIN_H

#include <QObject>
#include <functional>

namespace mssm
{

class Graphics;

class Plugin : public QObject
{
    Q_OBJECT
public:
    explicit Plugin(QObject *parent);
   ~Plugin();

    virtual void call(int arg1, int arg2, const std::string& arg3) = 0;

private:
    virtual bool shouldDelete() = 0;
    virtual void update(std::function<void(int, int, int, const std::string&)> sendEvent) = 0;
signals:

public slots:

    friend class Graphics;
};

}
#endif // PLUGIN_H
