/****************************************************************************
** Meta object code from reading C++ file 'krang2d.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../examples/krang2d/krang2d.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'krang2d.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Krang2DPlant_t {
    QByteArrayData data[8];
    char stringdata0[73];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Krang2DPlant_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Krang2DPlant_t qt_meta_stringdata_Krang2DPlant = {
    {
QT_MOC_LITERAL(0, 0, 12), // "Krang2DPlant"
QT_MOC_LITERAL(1, 13, 6), // "update"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 15), // "QVector<Scalar>"
QT_MOC_LITERAL(4, 37, 5), // "state"
QT_MOC_LITERAL(5, 43, 7), // "control"
QT_MOC_LITERAL(6, 51, 9), // "state_ref"
QT_MOC_LITERAL(7, 61, 11) // "control_ref"

    },
    "Krang2DPlant\0update\0\0QVector<Scalar>\0"
    "state\0control\0state_ref\0control_ref"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Krang2DPlant[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    4,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3, 0x80000000 | 3, 0x80000000 | 3,    4,    5,    6,    7,

       0        // eod
};

void Krang2DPlant::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Krang2DPlant *_t = static_cast<Krang2DPlant *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->update((*reinterpret_cast< const QVector<Scalar>(*)>(_a[1])),(*reinterpret_cast< const QVector<Scalar>(*)>(_a[2])),(*reinterpret_cast< const QVector<Scalar>(*)>(_a[3])),(*reinterpret_cast< const QVector<Scalar>(*)>(_a[4]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Krang2DPlant::*_t)(const QVector<Scalar> & , const QVector<Scalar> & , const QVector<Scalar> & , const QVector<Scalar> & ) const;
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Krang2DPlant::update)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject Krang2DPlant::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Krang2DPlant.data,
      qt_meta_data_Krang2DPlant,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Krang2DPlant::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Krang2DPlant::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Krang2DPlant.stringdata0))
        return static_cast<void*>(const_cast< Krang2DPlant*>(this));
    if (!strcmp(_clname, "Plant<double,8,2>"))
        return static_cast< Plant<double,8,2>*>(const_cast< Krang2DPlant*>(this));
    return QObject::qt_metacast(_clname);
}

int Krang2DPlant::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void Krang2DPlant::update(const QVector<Scalar> & _t1, const QVector<Scalar> & _t2, const QVector<Scalar> & _t3, const QVector<Scalar> & _t4)const
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(const_cast< Krang2DPlant *>(this), &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
