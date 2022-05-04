#include "threedrpui.h"
#include "viewer.h"

int main(int argc, char* argv[]){
    QApplication a(argc, argv);
    ThreeDRPUI w;
    w.show();
    return a.exec();
}