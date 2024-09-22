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

#include <orwell/Controller.hpp>

namespace orwell
{

    /**
     * @brief base implementation of a Controller tracking a given Reference
     * @details
     *
     * @tparam Reference target to be tracked
     * @tparam DerivedController either TorqueController, PositionController or VelocityController
     */
    template <class Reference, class DerivedController>
    class ReferenceController : public DerivedController
    {
      public:
        /**
         * @brief
         * @details loads configuration parameters
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        ReferenceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         */
        virtual ~ReferenceController();

        /**
         * @brief set Reference
         * @details
         *
         * @param reference target to be tracked
         */
        void setReference(const Reference &reference);

        /**
         * @brief provide access to the Reference to derived classes
         * @details
         * @return Reference
         */
        const Reference &getReference() const;

      protected:
      private:
        /// target to be tracked
        Reference reference_;

      public:
        /// typedef
        using Ptr = std::shared_ptr<ReferenceController>;
    };

}  // namespace orwell

#include <orwell/ReferenceController.hxx>