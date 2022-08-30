CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

TARGET = pdphgs

DEFINES += BALAS_SIMONETTI

#message([`hg -R $$PWD/.. | grep parent`])

HG_VER=$$system("git log --max-count=1 --abbrev-commit $$PWD/.. | grep -m 1 -oP '(?<=commit ).*'")
HG_VER_MSG=$$system("git log --max-count=1 --pretty=oneline $$PWD/..")


DEFINES += BUILD_PDP_VERSION='\'$$HG_VER\''
DEFINES += BUILD_PDP_VERSION_MSG='\'$$HG_VER_MSG\''

SOURCES += \
    pdp/pdprouteinfo.cpp \
    pdp/pdpsolution.cpp \
    utils/random.cpp \
    utils/application.cpp \
    pdp/pdproute.cpp \
    pdp/pdpinstance.cpp \
    pdp/pdpeducate.cpp \
    pdp/moves/pdprelocatemove.cpp \
    pdp/moves/pdporoptmove.cpp \
    pdp/moves/pdpmoveevaluation.cpp \
    pdp/moves/pdpbsmove.cpp \
    pdp/moves/pdp4optmove.cpp \
    pdp/moves/pdp2optmove.cpp \
    pdp/moves/pdp2koptmove.cpp \
    pdp/moves/balas_simonetti/util.cpp \
    pdp/moves/balas_simonetti/bsgraph.cpp \
    pdp/moves/balas_simonetti/bscache.cpp \
    pdp/instancereader.cpp \
    main.cpp \
    hgsadc/adcpopulation.cpp \
    hgsadc/hgsadc.cpp


HEADERS += \
    hgsadc/problem.h \
    hgsadc/solution.h \
    pdp/pdpnode.h \
    pdp/pdpsolution.h \
    utils/random.h \
    utils/application.h \
    pdp/pdproute.h \
    pdp/pdpinstance.h \
    pdp/pdpeducate.h \
    pdp/moves/pdprelocatemove.h \
    pdp/moves/pdporoptmove.h \
    pdp/moves/pdpmoveevaluation.h \
    pdp/moves/pdpmove.h \
    pdp/moves/pdpbsmove.h \
    pdp/moves/pdp4optmove.h \
    pdp/moves/pdp2optmove.h \
    pdp/moves/pdp2koptmove.h \
    pdp/moves/balas_simonetti/util.h \
    pdp/moves/balas_simonetti/bsgraph.h \
    pdp/moves/balas_simonetti/bscache.h \
    pdp/instancereader.h \
    pdp/pdprouteinfo.h \
    hgsadc/adcpopulation.h \
    hgsadc/hgsadc.h


LIBS += -lboost_program_options
LIBS += -lboost_filesystem
LIBS += -lboost_system
LIBS += -lboost_regex

QMAKE_CXXFLAGS += -std=c++0x

CONFIG(debug, debug|release) {
    QMAKE_CXXFLAGS += -Og
}else {
    QMAKE_CXXFLAGS_RELEASE -= -O2 -mtune=generic
    QMAKE_CXXFLAGS_RELEASE += -O3 -m64 -march=native
}


DISTFILES +=
