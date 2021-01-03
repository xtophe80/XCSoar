/*
 * Copyright 2012-2021 Max Kellermann <max.kellermann@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "EventPipe.hxx"
#include "Error.hxx"

#include <cassert>
#include <cstdint>

EventPipe::EventPipe()
{
#ifdef __linux__
	if (!r.CreateEventFD())
		throw MakeErrno("eventfd() has failed");
#else
	if (!UniqueFileDescriptor::CreatePipeNonBlock(r, w))
		throw MakeErrno("pipe() has failed");
#endif
}

void
EventPipe::Write() noexcept
{
#ifdef __linux__
	static constexpr uint64_t value = 1;
	r.Write(&value, sizeof(value));
#else
	static constexpr char dummy = 0;
	w.Write(&dummy, 1);
#endif
}

bool
EventPipe::Read() noexcept
{
#ifdef __linux__
	uint64_t value;
	return r.Read(&value, sizeof(value)) > 0;
#else
	char buffer[256];
	ssize_t nbytes = r.Read(buffer, sizeof(buffer));
	return nbytes > 0;
#endif
}