/****************************************************************************
** Meta object code from reading C++ file 'RosComunication.h'
**
** Created: Wed Aug 1 18:30:46 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/RosComunication.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RosComunication.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RosComunication[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   16,   16,   16, 0x05,
      31,   28,   16,   16, 0x05,
      93,   57,   16,   16, 0x05,

 // slots: signature, parameters, type, tag, flags
     116,   16,   16,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RosComunication[] = {
    "RosComunication\0\0rosQuits()\0id\0"
    "rfidRecieved(std::string)\0"
    "destroyedFactories,isTowerDestroyed\0"
    "towersUpdate(int,bool)\0quitNow()\0"
};

void RosComunication::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RosComunication *_t = static_cast<RosComunication *>(_o);
        switch (_id) {
        case 0: _t->rosQuits(); break;
        case 1: _t->rfidRecieved((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 2: _t->towersUpdate((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 3: _t->quitNow(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData RosComunication::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject RosComunication::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_RosComunication,
      qt_meta_data_RosComunication, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RosComunication::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RosComunication::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RosComunication::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RosComunication))
        return static_cast<void*>(const_cast< RosComunication*>(this));
    return QThread::qt_metacast(_clname);
}

int RosComunication::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void RosComunication::rosQuits()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void RosComunication::rfidRecieved(std::string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void RosComunication::towersUpdate(int _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
