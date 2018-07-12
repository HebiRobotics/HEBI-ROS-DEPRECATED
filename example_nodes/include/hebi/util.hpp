#pragma once

/**
 * This file defines helper macros to make the other files more readable.
 */

#define HEBI_DISABLE_COPY_MOVE(Class) \
/* Disable copy constructor. */ \
Class(const Class& other) = delete; \
/* Disable move constructor. */ \
Class(const Class&& other) = delete; \
/* Disable copy assigment operator. */ \
Class& operator= (const Class& other) = delete; \
/* Disable move assigment operator. */ \
Class& operator= (const Class&& other) = delete;

#define HEBI_DISABLE_COPY(Class) \
/* Disable copy constructor. */ \
Class(const Class& other) = delete; \
/* Disable copy assigment operator. */ \
Class& operator= (const Class& other) = delete;
