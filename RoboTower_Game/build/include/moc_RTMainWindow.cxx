/****************************************************************************
** Meta object code from reading C++ file 'RTMainWindow.h'
**
** Created: Wed Aug 1 18:30:47 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/RTMainWindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RTMainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RTMainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      43,   14,   13,   13, 0x0a,
      83,   66,   13,   13, 0x0a,
     115,  103,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RTMainWindow[] = {
    "RTMainWindow\0\0factoryNumber,destroyedTower\0"
    "updateTowers(int,bool)\0timeToLive,score\0"
    "updateData(int,int)\0card,status\0"
    "updateCardStatus(int,bool)\0"
};

void RTMainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RTMainWindow *_t = static_cast<RTMainWindow *>(_o);
        switch (_id) {
        case 0: _t->updateTowers((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 1: _t->updateData((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->updateCardStatus((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData RTMainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject RTMainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_RTMainWindow,
      qt_meta_data_RTMainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RTMainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RTMainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RTMainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RTMainWindow))
        return static_cast<void*>(const_cast< RTMainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int RTMainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
