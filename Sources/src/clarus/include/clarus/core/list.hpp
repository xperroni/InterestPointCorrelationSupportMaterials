/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Clarus.

Clarus is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Clarus is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Clarus. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CLARUS_CORE_LIST_HPP
#define CLARUS_CORE_LIST_HPP

#include <clarus/core/types.hpp>

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace clarus {
    template<class T> class List;

    template<class T, class F> void sort(List<T> &list);

    template<class T, class F> void sort(List<T> &list, F cmp);

    template<class T> List<T> sorted(const List<T> &list);

    template<class T, class F> List<T> sorted(const List<T> &list, F cmp);
}

template<class T> class clarus::List {
    typedef std::vector<T> Buffer;

    boost::shared_ptr<Buffer> buffer;

    inline int roll(int index) const {
        return (index < 0 ? ((int) size()) + index : index);
    }

public:
    List();

    List(size_t length);

    List(size_t length, const T &value);

    List(const Buffer &that);

    /*
    Virtual destructor. Enforces polymorphism. Do not remove.
    */
    virtual ~List();

    /**
    \brief Appends the given value to this list, then returns the updated list.
    */
    List operator , (const T &value);

    T &operator [] (int index);

    const T &operator [] (int index) const;

    bool operator == (const List &that) const;

    bool operator != (const List &that) const;

    bool operator < (const List &that) const;

    Buffer &operator * ();

    const Buffer &operator * () const;

    Buffer *operator -> ();

    const Buffer *operator -> () const;

    /**
    \brief Returns a subset of this list, from index \c a up to <tt>b - 1</tt>.

    Throws a <tt>std::runtime_error</tt> if the index range is illegal.
    */
    List operator () (int a, int b) const;

    T &at(int index);

    const T &at(int index) const;

    T &append(const T &value = T());

    List clone(bool deep = false) const;

    void extend(const List &that);

    void clear();

    bool contains(const T &value) const;

    bool empty() const;

    T remove(int index);

    T &first();

    const T &first() const;

    T &last();

    const T &last() const;

    size_t size() const;
};

#include <clarus/core/list_iterator.hpp>
#include <clarus/core/list_iterator_const.hpp>

template<class T> clarus::List<T>::List():
  buffer(new Buffer())
{
    // Nothing to do.
}

template<class T> clarus::List<T>::List(size_t length):
  buffer(new Buffer())
{
  buffer->reserve(length);
  for (int i = 0; i < length; i++)
    append();
}

template<class T> clarus::List<T>::List(size_t length, const T &value):
    buffer(new Buffer(length, value))
{
    // Nothing to do.
}

template<class T> clarus::List<T>::List(const Buffer &that):
    buffer(new Buffer(that))
{
    // Nothing to do.
}

template<class T> clarus::List<T>::~List() {
    // Nothing to do.
}

template<class T> clarus::List<T> clarus::List<T>::operator , (const T &value) {
    append(value);
    return *this;
}

template<class T> T &clarus::List<T>::operator [] (int index) {
    return buffer->at(roll(index));
}

template<class T> const T &clarus::List<T>::operator [] (int index) const {
    return buffer->at(roll(index));
}

template<class T> bool clarus::List<T>::operator == (const List &that) const {
    if (size() != that.size()) {
        return false;
    }

    for (ListIteratorConst<T> i(*this), j(that); i.more();) {
        const T &a = i.next();
        const T &b = j.next();
        if (a != b) {
            return false;
        }
    }

    return true;
}

template<class T> bool clarus::List<T>::operator != (const List &that) const {
    return !(*this == that);
}

template<class T> bool clarus::List<T>::operator < (const List &that) const {
    for (ListIteratorConst<T> i(*this), j(that); i.more() && j.more();) {
        const T &a = i.next();
        const T &b = j.next();
        if (a < b) {
            return true;
        }
        else if (b < a) {
            return false;
        }
    }

    return (size() < that.size());
}

template<class T> typename clarus::List<T>::Buffer& clarus::List<T>::operator * () {
    return *buffer;
}

template<class T> const typename clarus::List<T>::Buffer& clarus::List<T>::operator * () const {
    return *buffer;
}

template<class T> typename clarus::List<T>::Buffer *clarus::List<T>::operator -> () {
    return buffer.get();
}

template<class T> const typename clarus::List<T>::Buffer *clarus::List<T>::operator -> () const {
    return buffer.get();
}

template<class T> clarus::List<T> clarus::List<T>::operator () (int a, int b) const {
    int n = size();
    if (a >= b || (b - a - 1) >= n) {
        std::string error =
            std::string("Illegal range ") +
            "(" + types::to_string(a) + ", " + types::to_string(b) + ") "
            "for List of size (" + types::to_string(n) + ")"
        ;

        throw std::runtime_error(error);
    }

    List subset;
    for (int i = a; i < b; i++) {
        subset.append(at(i));
    }

    return subset;
}

template<class T> T &clarus::List<T>::at(int index) {
    return buffer->at(roll(index));
}

template<class T> const T &clarus::List<T>::at(int index) const {
    return buffer->at(roll(index));
}

template<class T> T &clarus::List<T>::append(const T &value) {
    buffer->push_back(value);
    return buffer->back();
}

template<class T> clarus::List<T> clarus::List<T>::clone(bool deep) const {
    if (deep) {
        return List(Buffer(*buffer));
    }
    else {
        return List(*buffer);
    }
}

//template<> clarus::List<cv::Mat> clarus::List<cv::Mat>::clone(bool deep) const;

template<class T> void clarus::List<T>::extend(const List &that) {
    for (ListIteratorConst<T> i(that); i.more();) {
        append(i.next());
    }
}

template<class T> void clarus::List<T>::clear() {
    buffer->clear();
}

template<class T> bool clarus::List<T>::contains(const T &value) const {
    for (ListIteratorConst<T> i(*this); i.more();) {
        if (i.next() == value) {
            return true;
        }
    }

    return false;
}

template<class T> bool clarus::List<T>::empty() const {
    return (size() == 0);
}

template<class T> T clarus::List<T>::remove(int index) {
    index = roll(index);
    if (index < 0 || index >= size()) {
        throw std::runtime_error("Invalid index: " + types::to_string(index));
    }

    typename std::vector<T>::iterator i = buffer->begin();
    for (int j = 0; j < index; ++j) {
        ++i;
    }

    T value = *i;
    buffer->erase(i);
    return value;
}

template<class T> T &clarus::List<T>::first() {
    return buffer->front();
}

template<class T> const T &clarus::List<T>::first() const {
    return buffer->front();
}

template<class T> T &clarus::List<T>::last() {
    return buffer->back();
}

template<class T> const T &clarus::List<T>::last() const {
    return buffer->back();
}

template<class T> size_t clarus::List<T>::size() const {
    return buffer->size();
}

template<class T> void clarus::sort(List<T> &list) {
    std::vector<T> &v = *list;
    std::sort(v.begin(), v.end());
}

template<class T, class F> void clarus::sort(List<T> &list, F cmp) {
    std::vector<T> &v = *list;
    std::sort(v.begin(), v.end(), cmp);
}

template<class T> clarus::List<T> clarus::sorted(const List<T> &list) {
    List<T> cloned = list.clone();
    sort(cloned);
    return cloned;
}

template<class T, class F> clarus::List<T> clarus::sorted(const List<T> &list, F cmp) {
    List<T> cloned = list.clone();
    sort(cloned, cmp);
    return cloned;
}

template<class T> std::ostream &operator << (std::ostream &out, const clarus::List<T> &list) {
    if (list.size() == 0) {
        out << "[]";
        return out;
    }

    out << "[";
    for (clarus::ListIteratorConst<T> i(list);;) {
        out << i.next();
        if (!i.more()) {
            break;
        }

        out << ", ";
    }

    out << "]";

    return out;
}

template<class T> std::istream &operator >> (std::istream &in, clarus::List<T> &list) {
    char sep = '\0';
    in >> sep;
    if (sep != '[') {
        throw std::runtime_error("Error reading List from stream: separator character '[' not found");
    }

    for (;;) {
        T &value = list.append();
        in >> value;

        in >> sep;
        if (sep == ']') {
            return in;
        }
    }
}

#endif
