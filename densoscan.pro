QT += core gui
QT += charts
QT += multimedia
QT += widgets

CONFIG += c++11

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

RESOURCES += \
    densoscan.qrc
