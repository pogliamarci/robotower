/****************************************************************************
** Meta object code from reading C++ file 'GameControl.h'
**
** Created: Wed Aug 1 17:35:42 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/GameControl.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GameControl.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GameControl[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      30,   13,   12,   12, 0x05,
      72,   60,   12,   12, 0x05,
     100,   12,   12,   12, 0x05,

 // slots: signature, parameters, type, tag, flags
     113,  110,   12,   12, 0x0a,
     138,   12,   12,   12, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GameControl[] = {
    "GameControl\0\0timeToLive,score\0"
    "updatedTimeAndPoints(int,int)\0rfid,status\0"
    "updatedRfidStatus(int,bool)\0endGame()\0"
    "id\0disableRFID(std::string)\0quitNow()\0"
};

void GameControl::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GameControl *_t = static_cast<GameControl *>(_o);
        switch (_id) {
        case 0: _t->updatedTimeAndPoints((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->updatedRfidStatus((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->endGame(); break;
        case 3: _t->disableRFID((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 4: _t->quitNow(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GameControl::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GameControl::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_GameControl,
      qt_meta_data_GameControl, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GameControl::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GameControl::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GameControl::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GameControl))
        return static_cast<void*>(const_cast< GameControl*>(this));
    return QThread::qt_metacast(_clname);
}

int GameControl::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void GameControl::updatedTimeAndPoints(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GameControl::updatedRfidStatus(int _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void GameControl::endGame()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
