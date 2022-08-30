CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

TARGET = pdprr

QMAKE_CXXFLAGS += -std=c++0x

SOURCES += \
        instance.cpp \
        operators.cpp \
        solver.cpp \
        application.cpp \
        random.cpp

HEADERS += \
    instance.h \
    operators.h \
    solver.h \
    application.h \
    random.h

LIBS += -lboost_program_options
LIBS += -lboost_filesystem
LIBS += -lboost_system
LIBS += -lboost_regex

CONFIG(debug, debug|release) {
  #QMAKE_CXXFLAGS += -Og
  #DEFINES += BRUTE_FORCE_ASSERTION
}else {
  QMAKE_CXXFLAGS_RELEASE += -O3 -m64 -march=native
}
