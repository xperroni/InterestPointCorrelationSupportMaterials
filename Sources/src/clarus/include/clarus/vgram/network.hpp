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

#ifndef CLARUS_VGRAM_NETWORK_HPP
#define CLARUS_VGRAM_NETWORK_HPP

#include <clarus/vgram/layer.hpp>

namespace vgram {
    template<class W> class network;
}

template<class W> class vgram::network {
    typedef typename W::X X;

    typedef typename W::Y Y;

    typedef typename W::B B;

    typedef typename W::G G;

    typedef typename W::J J;

    typedef typename W::L L;

    typedef typename W::N N;

    typedef typename W::P P;

    typedef typename W::alpha alpha;

    typedef typename W::beta beta;

    typedef typename W::gamma gamma;

    typedef typename W::omega omega;

    typedef output<Y> O;

    P p;

    N n;

public:
    network(const P &p);

    O get(const X &x) const;

    void set(const X &x, const Y &y);
};

template<class W> vgram::network<W>::network(const P &_p):
    p(_p),
    n(p)
{
    // Nothing to do.
}


template<class W> typename vgram::network<W>::O vgram::network<W>::get(const X &x) const {
    alpha a(p, x);
    omega w(a);

    for (gamma c(a); c.more(); c.next()) {
        const J &j = c.input();
        const L &l = c.neuron();
        const B &i = a(j, l);
        output<G> o = n.get(l, i);
        if (!o.empty()) {
            const G &g = o.value();
            size_t e = o.error();
            w.add(j, l, g, e);
        }
    }

    return w();
}

template<class W> void vgram::network<W>::set(const X &x, const Y &y) {
    alpha a(p, x);
    beta b(a, y);

    for (gamma c(a, b); c.more(); c.next()) {
        const J &j = c.input();
        const L &l = c.neuron();
        const B &i = a(j, l);
        const G &g = b(j);
        n.set(l, i, g);
    }
}

#endif
