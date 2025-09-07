cmake_minimum_required(VERSION 3.10)

# Nome del progetto e versione di C++
project(densoscan LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Trova i pacchetti necessari con pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(SANE_BACKENDS REQUIRED sane-backends)
pkg_check_modules(FMT REQUIRED fmt)
pkg_check_modules(LIBPNG REQUIRED libpng)
pkg_check_modules(OPENCV4 REQUIRED opencv4)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts Multimedia)

# Includi le directory dove si trovano gli header
include_directories(
    ${SANE_BACKENDS_INCLUDE_DIRS}
    ${FMT_INCLUDE_DIRS}
    ${LIBPNG_INCLUDE_DIRS}
    ${OPENCV4_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/build
)

# File sorgenti
set(SOURCES
    src/main.cpp
    src/densoscan.cpp
    src/options.cpp
    src/scanner.cpp
    qrc_densoscan.cpp
)

# File header
set(HEADERS
    src/densoscan.h
    src/options.h
    src/scanner.h
)

# File di UI (interfaccia utente)
set(UI_FILES
    src/densoscan.ui
    src/options.ui
)

# File di risorse
set(RESOURCE_FILES
    src/densoscan.qrc
)

# Aggiungi il file di icone come risorsa
set(RC_ICONS
    icon/icon.ico
)

# Genera codice per i file UI e risorse (Qt6 gestisce automaticamente con AUTOUIC/AUTORCC)
set(UI_HEADERS)
set(RESOURCE_SOURCES)

# Compila l'eseguibile
add_executable(densoscan ${SOURCES} ${HEADERS} ${UI_HEADERS} ${RESOURCE_SOURCES})

# Collega le librerie di Qt6 e altre librerie
target_link_libraries(densoscan
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Charts
    Qt6::Multimedia
    ${SANE_BACKENDS_LIBRARIES}
    ${FMT_LIBRARIES}
    ${LIBPNG_LIBRARIES}
    ${OPENCV4_LIBRARIES}
    pthread
)

# Opzionale: se hai un file icona di risorsa
if(RC_ICONS)
    set_source_files_properties(${RC_ICONS} PROPERTIES HEADER_FILE_ONLY TRUE)
    target_sources(densoscan PRIVATE ${RC_ICONS})
endif()
