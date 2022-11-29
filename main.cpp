#include "sptest.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    sptest w;
    w.show();
    return a.exec();
}
