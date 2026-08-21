#ifndef _LINK_SERIAL_H
#define _LINK_SERIAL_H

#include <gambatte.h>
#include <stdint.h>

#include "link_interface.h"

/* A Game Boy link cable carried by the frontend's link bus.
 *
 * NetSerial's job done without a socket. The network version works and is the
 * only way two RetroArch instances on two machines can be joined, but it pays
 * for that with a blocking socket read in the middle of the emulation loop and
 * with a wire protocol that has no shared clock: whichever end asks first waits
 * for the other, and how long it waits is wall-clock luck. Two cores in ONE
 * process can do better, and have to -- a room where every handheld's picture
 * is drawn into the same frame cannot afford one of them stalling on a socket,
 * and a netplay session replaying the same inputs on two peers has to reach the
 * same answer both times.
 *
 * What a core cannot do is find the other core by itself. A frontend that runs
 * several at once generally loads each from its own copy of the shared library,
 * so two Game Boys share no globals whatsoever; the frontend is the only thing
 * they have in common, which is why the bus lives there and is reached through
 * RETRO_ENVIRONMENT_GET_LINK_INTERFACE.
 *
 * WHAT CROSSES. A Game Boy's serial port is two machines and one wire, and
 * unlike a Game Boy Advance's it has no notion of a parent: whichever unit
 * selects the internal clock (SC bit 0) drives the transfer, and that is a
 * choice the GUEST makes, both ends being identical hardware. So there are only
 * two messages. A machine says what its serial register holds and whether it is
 * armed (LS_STATE), and a machine that clocks a transfer says so (LS_CLOCK).
 * Everything else -- which end is master, how long a byte takes, when the
 * interrupt fires -- each end decides for itself out of what it already knows,
 * exactly as the hardware does.
 *
 * WHAT DOES NOT. The four-player adapter, the Super Game Boy's link and the
 * Game Boy Printer are not carried: gambatte emulates none of them, and a bus
 * that quietly joined three Game Boys would produce transfers no real cable
 * could. Anything past a pair is refused, and both machines fall back to the
 * 0xFF an unanswered cable gives.
 */
class LinkSerial : public gambatte::SerialIO
{
	public:
		LinkSerial();
		virtual ~LinkSerial();

		/* Remember the bus. Called once, from retro_init, whether or not the
		 * cable is ever switched on: the interface stays valid for the life of
		 * the core and holding it costs nothing. */
		void setInterface(const struct retro_link_interface *link, unsigned port);
		bool available() const { return link_ != 0; }

		bool start();
		void stop();
		bool started() const { return attached_; }

		/* A reset puts the machine's cycle counter back to zero, which the bus
		 * would read as an enormous jump backwards. Re-anchor instead, and keep
		 * the accumulated total moving forward. */
		void reset();

		/* Called from retro_run before each chunk of emulation. Returns how many
		 * stereo samples this machine may produce before the next rendezvous,
		 * which is what stops it running past a peer, and `wanted` unchanged
		 * when nothing is cabled to it. */
		unsigned advance(unsigned wanted);

		/* And after each chunk, with what it actually produced.
		 *
		 * Samples are the frontend's own measure of emulated time and they do
		 * not lie: there are 35112 in a video frame whether or not the machine
		 * is in double speed, so four link ticks to the sample holds in both.
		 * The cycle counter is the finer clock and is read inside the chunk;
		 * this is the coarse one that says where a chunk ENDS, which the cycle
		 * counter cannot, because nothing samples it between the last event of
		 * one chunk and the first of the next. */
		void produced(unsigned samples);

		unsigned peers() const { return peers_; }

		/* SerialIO. */
		virtual bool check(unsigned char out, unsigned char &in, bool &fastCgb);
		virtual unsigned char send(unsigned char data, bool fastCgb);
		virtual void tick(unsigned long cc, bool doubleSpeed,
		                  unsigned char sb, unsigned char sc);

	private:
		/* A message from a peer, held until this machine's clock reaches the
		 * tick it was stamped with.
		 *
		 * Acting on one the moment it arrives is the whole difference between a
		 * link that replays and one that does not: WHEN it arrives is a
		 * wall-clock accident, when its tick comes round is not. */
		struct Pending
		{
			uint64_t tick;
			unsigned char type;
			unsigned char sb;
			unsigned char flags;
		};

		enum { PENDING_MAX = 16 };

		void publish(unsigned char sb, unsigned char sc);
		void pump();
		void queue(const Pending &msg);
		void apply(const Pending &msg);
		void applyDue();
		void refreshPeers();
		void note(unsigned long cc, bool doubleSpeed);
		uint64_t transferTicks(bool fastCgb) const;
		void wire(uint64_t tick, unsigned char type, unsigned char sb, unsigned char flags);

		const struct retro_link_interface *link_;
		unsigned port_;
		retro_link_handle_t handle_;
		bool attached_;

		int selfId_;
		unsigned peers_;

		/* This machine's position on the link timeline, in ticks of a clock
		 * running at twice the Game Boy's normal 4.194304 MHz.
		 *
		 * Twice, so that double speed is expressible. gambatte's cycle counter
		 * is not a real-time clock: it advances one tick per CPU cycle and the
		 * CPU cycle itself halves in double speed, which is exactly what makes a
		 * serial transfer's fixed cycle count come out twice as fast on a Game
		 * Boy Color running flat out, the way the hardware does. Handing those
		 * raw cycles to the bus as a fixed rate would have it believe a machine
		 * in double speed is running at twice real time and throttle it to half,
		 * so the doubling is undone here instead. */
		uint64_t now_;
		uint64_t floor_;
		unsigned long lastRaw_;
		bool haveRaw_;
		bool doubleSpeed_;

		/* How far ahead of itself this machine promises not to originate
		 * anything, and how far it asks to run between rendezvous.
		 *
		 * Both follow the transfer speed, because the shortest transfer does: a
		 * normal one is 8192 ticks and a Game Boy Color fast one is 256, and a
		 * horizon that suited the first would swallow the second whole. */
		uint64_t horizon_;
		uint64_t grain_;
		uint64_t grant_;
		bool fastSeen_;

		/* The peer's serial register, as of the last state it published that
		 * this machine's clock has reached. */
		unsigned char peerSb_;
		bool peerArmed_;

		/* What this machine last told the bus, so a game polling its own serial
		 * registers in a loop does not flood the wire with identical states. */
		unsigned char lastSb_;
		unsigned char lastFlags_;
		bool published_;

		/* Whether this machine has rendezvoused since the last change of
		 * membership, and the tick it stops re-announcing itself at.
		 *
		 * Both exist because seating a cable rebuilds the bus, and a rebuild takes
		 * every endpoint off its timeline until it publishes again. A message to a
		 * peer in that state has nowhere to land and the bus drops it, so a driver
		 * that announces itself ONCE on a membership change announces into
		 * nothing, and whichever machine noticed the cable first is never heard
		 * from at all. Nothing about that looks wrong from inside a core: the peer
		 * count is right, the cable is seated, and no call fails.
		 *
		 * A DEADLINE rather than a flag cleared when the peer answers, because WHEN
		 * an answer arrives is wall-clock luck and this decides whether a message
		 * goes out at all. Under replicated netplay both peers run both machines and
		 * have to reach the same answer, so a branch taken on arrival would have one
		 * peer announce where the other stayed quiet, and the two copies of the same
		 * Game Boy would part company over a cable neither player touched. A tick
		 * this machine's own clock passes is the same tick on both. */
		bool anchored_;
		uint64_t reannounceUntil_;

		/* A transfer this machine is in the middle of, whether it clocked it or
		 * was clocked: while one runs, the serial register is being shifted
		 * through and holds nothing a peer should read. */
		uint64_t busyUntil_;
		bool busy_;

		/* A transfer a peer has clocked, waiting for this machine's own clock to
		 * reach the moment it starts. */
		bool clockReady_;
		uint64_t clockAt_;
		unsigned char clockSb_;
		bool clockFast_;

		Pending pending_[PENDING_MAX];
		unsigned pendingCount_;
};

#endif
