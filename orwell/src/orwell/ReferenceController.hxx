/*
    Copyright (c) Tobias Löw
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of orwell.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <orwell/ReferenceController.hpp>

namespace orwell
{

    template <class Reference, class DerivedController>
    ReferenceController<Reference, DerivedController>::ReferenceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : DerivedController(interface, name)
    {}

    template <class Reference, class DerivedController>
    ReferenceController<Reference, DerivedController>::~ReferenceController() = default;

    template <class Reference, class DerivedController>
    void ReferenceController<Reference, DerivedController>::setReference(const Reference &reference)
    {
        reference_ = reference;
    }

    template <class Reference, class DerivedController>
    const Reference &ReferenceController<Reference, DerivedController>::getReference() const
    {
        return reference_;
    }

}  // namespace orwell