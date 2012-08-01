/****************************************************************************
** Meta object code from reading C++ file 'RosPublisher.h'
**
** Created: Wed Aug 1 13:21:08 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/RosPublisher.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RosPublisher.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RosPublisher[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      25,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RosPublisher[] = {
    "RosPublisher\0\0rosQuits()\0quitNow()\0"
};

void RosPublisher::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RosPublisher *_t = static_cast<RosPublisher *>(_o);
        switch (_id) {
        case 0: _t->rosQuits(); break;
        case 1: _t->quitNow(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData RosPublisher::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject RosPublisher::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_RosPublisher,
      qt_meta_data_RosPublisher, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RosPublisher::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RosPublisher::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RosPublisher::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RosPublisher))
        return static_cast<void*>(const_cast< RosPublisher*>(this));
    return QThread::qt_metacast(_clname);
}

int RosPublisher::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void RosPublisher::rosQuits()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
