QT       += core gui
QT += charts


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    densoscan.cpp \
    options.cpp \
    scanner.cpp

HEADERS += \
    densoscan.h \
    options.h \
    scanner.h

FORMS += \
    densoscan.ui \
    options.ui

QMAKE_CXXFLAGS += -pthread

CONFIG    += link_pkgconfig
PKGCONFIG += sane-backends
PKGCONFIG += fmt
PKGCONFIG += libpng
PKGCONFIG += opencv4
PKGCONFIG += libpng

RC_ICONS = icon/icon.ico

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    densoscan.qrc
