QT -= gui
QT += widgets
CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0




INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
              /usr/local/include/opencv2 \
#             /home/lyn/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122/Galaxy_camera/inc

LIBS += /usr/local/lib/libopenc*
#LIBS += /home/lyn/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122/Galaxy_camera/inc/Gx*
#LIBS += /home/lyn/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122/Galaxy_camera/inc/Dx*
#LIBS += -lgxiapi
LIBS += /lib/libMVSDK.so

SOURCES += \
    ImageConsProd.cpp \
    main.cpp \
    armordetection.cpp \
    rp_kalman.cpp \
    serialport.cpp \
    CRC_Check.cpp \
    Energy.cpp \
#    object_predict.cpp \
    run_vedio.cpp \
    numpredict.cpp \
    antispinning.cpp \
    deal_drop_frame.cpp \
    save_video.cpp



HEADERS += \
#    DxImageProc.h \
#    GxIAPI.h \
    ImageConsProd.h \
    armordetection.h \
    rp_kalman.h \
    serialport.h \
    CRC_Check.h \
    Energy.h \
#    object_predict.h
    numpredict.h \
    CameraApi.h \
    CameraStatus.h \
    CameraDefine.h \
    antispinning.h \
    deal_drop_frame.h \
    save_video.h
