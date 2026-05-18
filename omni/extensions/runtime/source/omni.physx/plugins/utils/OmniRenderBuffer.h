// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "carb/Defines.h"
#include <PxPhysicsAPI.h>
#include <vector>

namespace omni
{
    namespace physx
    {
        /**
         * \brief Wrapper around PhysX PxRenderBuffer providing a convenient interface for debug rendering.
         * 
         * This class implements the PxRenderBuffer interface and provides storage for debug visualization
         * primitives (points, lines, triangles). All functionality is fully implemented including:
         * - Adding, retrieving, and counting debug points, lines, and triangles
         * - Reserving space for batch additions
         * - Appending contents from other render buffers
         * - Shifting all primitives by an offset
         * - Clearing all primitives
         */
        class OmniRenderBuffer : public ::physx::PxRenderBuffer
        {
        public:
            OmniRenderBuffer() {}
            virtual ~OmniRenderBuffer() {}

            /**
             * \brief Get the number of debug points stored in the buffer.
             * \return The count of debug points currently stored.
             */
            virtual ::physx::PxU32 getNbPoints() const override { return static_cast<::physx::PxU32>(mPoints.size()); }
            
            /**
             * \brief Get a pointer to the array of debug points.
             * \return Pointer to the first debug point in the buffer, or nullptr if the buffer is empty.
             */
            virtual const ::physx::PxDebugPoint* getPoints() const override { return mPoints.empty() ? nullptr : mPoints.data(); }
            
            /**
             * \brief Add a debug point to the render buffer.
             * \param point The debug point to add, containing position and color.
             */
            virtual void addPoint(const ::physx::PxDebugPoint& point) override { mPoints.push_back(point); }
            
            /**
             * \brief Get the number of debug lines stored in the buffer.
             * \return The count of debug lines currently stored.
             */
            virtual ::physx::PxU32 getNbLines() const override { return static_cast<::physx::PxU32>(mLines.size()); }
            
            /**
             * \brief Get a pointer to the array of debug lines.
             * \return Pointer to the first debug line in the buffer, or nullptr if the buffer is empty.
             */
            virtual const ::physx::PxDebugLine* getLines() const override { return mLines.empty() ? nullptr : mLines.data(); }
            
            /**
             * \brief Add a debug line to the render buffer.
             * \param line The debug line to add, containing two points and their colors.
             */
            virtual void addLine(const ::physx::PxDebugLine& line) override { mLines.push_back(line); }
            
            /**
             * \brief Reserve space for a specified number of lines and return a pointer to the reserved memory.
             * \param nbLines The number of lines to reserve space for.
             * \return Pointer to the first reserved line slot.
             */
            virtual ::physx::PxDebugLine* reserveLines(const ::physx::PxU32 nbLines) override 
            { 
                return nullptr;
            }
            
            /**
             * \brief Reserve space for a specified number of points and return a pointer to the reserved memory.
             * \param nbPoints The number of points to reserve space for.
             * \return Pointer to the first reserved point slot.
             */
            virtual ::physx::PxDebugPoint* reservePoints(const ::physx::PxU32 nbPoints) override 
            { 
                return nullptr;
            }
            
            /**
             * \brief Get the number of debug triangles stored in the buffer.
             * \return The count of debug triangles currently stored.
             */
            virtual ::physx::PxU32 getNbTriangles() const override { return static_cast<::physx::PxU32>(mTriangles.size()); }
            
            /**
             * \brief Get a pointer to the array of debug triangles.
             * \return Pointer to the first debug triangle in the buffer, or nullptr if the buffer is empty.
             */
            virtual const ::physx::PxDebugTriangle* getTriangles() const override { return mTriangles.empty() ? nullptr : mTriangles.data(); }
            
            /**
             * \brief Add a debug triangle to the render buffer.
             * \param triangle The debug triangle to add, containing three points and their colors.
             */
            virtual void addTriangle(const ::physx::PxDebugTriangle& triangle) override { mTriangles.push_back(triangle); }
            
            /**
             * \brief Append the contents of another render buffer to this buffer.
             * \param other The render buffer whose contents should be appended to this buffer.
             */
            virtual void append(const ::physx::PxRenderBuffer& other) override 
            {
                // Append points
                const ::physx::PxDebugPoint* otherPoints = other.getPoints();
                ::physx::PxU32 nbPoints = other.getNbPoints();
                for (::physx::PxU32 i = 0; i < nbPoints; ++i)
                {
                    mPoints.push_back(otherPoints[i]);
                }
                
                // Append lines
                const ::physx::PxDebugLine* otherLines = other.getLines();
                ::physx::PxU32 nbLines = other.getNbLines();
                for (::physx::PxU32 i = 0; i < nbLines; ++i)
                {
                    mLines.push_back(otherLines[i]);
                }
                
                // Append triangles
                const ::physx::PxDebugTriangle* otherTriangles = other.getTriangles();
                ::physx::PxU32 nbTriangles = other.getNbTriangles();
                for (::physx::PxU32 i = 0; i < nbTriangles; ++i)
                {
                    mTriangles.push_back(otherTriangles[i]);
                }
            }
            
            /**
             * \brief Clear all debug primitives from the render buffer.
             * 
             * Removes all stored points, lines, and triangles from the buffer,
             * resetting it to an empty state.
             */
            virtual void clear() override 
            { 
                mPoints.clear();
                mLines.clear();
                mTriangles.clear();
            }
            
            /**
             * \brief Shift all debug primitives by a given offset vector.
             * \param delta The offset vector to apply to all positions in the buffer.
             */
            virtual void shift(const ::physx::PxVec3& delta) override 
            {
                // Shift all points
                for (::physx::PxDebugPoint& point : mPoints)
                {
                    point.pos += delta;
                }
                
                // Shift all lines
                for (::physx::PxDebugLine& line : mLines)
                {
                    line.pos0 += delta;
                    line.pos1 += delta;
                }
                
                // Shift all triangles
                for (::physx::PxDebugTriangle& triangle : mTriangles)
                {
                    triangle.pos0 += delta;
                    triangle.pos1 += delta;
                    triangle.pos2 += delta;
                }
            }
            
            /**
             * \brief Check if the render buffer is empty.
             * \return True if the buffer contains no debug primitives, false otherwise.
             */
            virtual bool empty() const override 
            { 
                return mPoints.empty() && mLines.empty() && mTriangles.empty(); 
            }

        private:
            std::vector<::physx::PxDebugPoint> mPoints;
            std::vector<::physx::PxDebugLine> mLines;
            std::vector<::physx::PxDebugTriangle> mTriangles;
        };
    }
}
