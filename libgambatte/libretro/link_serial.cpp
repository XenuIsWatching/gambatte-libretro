#include "link_serial.h"

#include <string.h>

#include "gambatte_log.h"

/* Names the emulated wire. Peers whose ids differ are never joined, which is
 * what keeps a Game Boy link lead out of a Game Boy Advance's socket: mGBA's
 * driver calls its cable "gba-sio-1" and the two protocols have nothing in
 * common but the shape of the connector. */
#define LINK_PROTOCOL "gb-sio-1"

/* Twice 4194304, the Game Boy's normal CPU clock. See the note on now_. */
#define LINK_CLOCK_RATE ((uint64_t)8388608)

/* Ticks in a serial transfer at normal speed, and at the Game Boy Color's
 * faster 262144 Hz rate. Both are what gambatte itself schedules -- 4096 and
 * 128 cycles -- doubled into link ticks. */
#define LINK_XFER_NORMAL ((uint64_t)8192)
#define LINK_XFER_FAST ((uint64_t)256)

/* Rendezvous roughly every quarter of a transfer.
 *
 * The horizon is how far two machines may drift apart, and it is also how stale
 * the peer's serial register can be when this one clocks a byte out of it. A
 * game arms its end -- writes SB, then SC -- and then waits, so the value is
 * settled well before it is read, and a quarter of a byte time (244us of a
 * millisecond) is comfortably inside the gap every link protocol leaves between
 * bytes. Real ones leave far more: a Pokemon trade moves a byte every few
 * milliseconds and Tetris one a frame.
 *
 * The floor is cost. Every grain is a rendezvous between two emulation threads,
 * so this cannot go much lower without both cores spending their time
 * synchronising instead of emulating. The fast figure is a quarter of a fast
 * transfer by the same reasoning and is thirty-two times as expensive; it is
 * paid only while a Game Boy Color is actually running its link at 256 kHz,
 * which almost nothing does. */
#define LINK_GRAIN_NORMAL ((uint64_t)2048)
#define LINK_GRAIN_FAST ((uint64_t)64)

/* How long a machine keeps saying what it is holding after the cable moves,
 * counted in rendezvous. The window has to outlast the peer reaching its own
 * first rendezvous after the rebuild, which is one; eight is that with room to
 * spare, and costs eight messages of eight bytes. */
#define LINK_REANNOUNCE_GRAINS 8

/* Bus message. Packed by hand rather than shipped as a struct: both ends are
 * the same build today, but protocol_id exists so that another Game Boy core
 * could speak this later, and by then a shared struct layout would be an
 * assumption nobody remembers having made. */
enum
{
	LS_STATE = 1,
	LS_CLOCK
};

enum
{
	LS_FLAG_ARMED = 1,
	LS_FLAG_FAST = 2
};

#define LS_MSG_SIZE 8

LinkSerial::LinkSerial()
	: link_(0)
	, port_(0)
	, handle_(0)
	, attached_(false)
	, selfId_(0)
	, peers_(0)
	, now_(0)
	, floor_(0)
	, lastRaw_(0)
	, haveRaw_(false)
	, doubleSpeed_(false)
	, horizon_(LINK_GRAIN_NORMAL)
	, grain_(LINK_GRAIN_NORMAL)
	, grant_(0)
	, fastSeen_(false)
	, peerSb_(0xFF)
	, peerArmed_(false)
	, lastSb_(0)
	, lastFlags_(0)
	, published_(false)
	, anchored_(false)
	, reannounceUntil_(0)
	, busyUntil_(0)
	, busy_(false)
	, clockReady_(false)
	, clockAt_(0)
	, clockSb_(0xFF)
	, clockFast_(false)
	, pendingCount_(0)
{
	memset(pending_, 0, sizeof(pending_));
}

LinkSerial::~LinkSerial()
{
	stop();
}

void LinkSerial::setInterface(const struct retro_link_interface *link, unsigned port)
{
	link_ = link;
	port_ = port;
}

bool LinkSerial::start()
{
	if (attached_)
		return true;
	if (!link_)
		return false;

	handle_ = link_->attach(port_, LINK_PROTOCOL, LINK_CLOCK_RATE);
	if (!handle_)
	{
		gambatte_log(RETRO_LOG_WARN, "Link cable: the frontend refused port %u\n", port_);
		return false;
	}

	attached_ = true;
	pendingCount_ = 0;
	published_ = false;
	anchored_ = false;
	reannounceUntil_ = 0;
	clockReady_ = false;
	busy_ = false;
	refreshPeers();
	gambatte_log(RETRO_LOG_INFO, "Link cable: attached to port %u, %u machine(s) on the wire\n",
	             port_, peers_);
	return true;
}

void LinkSerial::stop()
{
	if (!attached_)
		return;

	link_->detach(handle_);
	attached_ = false;
	anchored_ = false;
	reannounceUntil_ = 0;
	handle_ = 0;
	peers_ = 0;
	selfId_ = 0;
	pendingCount_ = 0;
	clockReady_ = false;
	busy_ = false;
}

void LinkSerial::reset()
{
	/* The bus must never see this machine's clock go backwards, so the total
	 * carries on from where it stood while the counter it is read from starts
	 * again from zero. */
	haveRaw_ = false;
	pendingCount_ = 0;
	published_ = false;
	anchored_ = false;
	reannounceUntil_ = 0;
	clockReady_ = false;
	busy_ = false;
	fastSeen_ = false;
	peerSb_ = 0xFF;
	peerArmed_ = false;
}

uint64_t LinkSerial::transferTicks(bool fastCgb) const
{
	uint64_t ticks = fastCgb ? LINK_XFER_FAST : LINK_XFER_NORMAL;

	/* gambatte schedules a transfer as a fixed number of CPU cycles, and a CPU
	 * cycle is half as long in double speed, so the same transfer takes half the
	 * link ticks. That is the hardware's behaviour, not an approximation of it:
	 * a Game Boy Color in double speed really does clock its serial port twice
	 * as fast. */
	return doubleSpeed_ ? (ticks >> 1) : ticks;
}

void LinkSerial::note(unsigned long cc, bool doubleSpeed)
{
	doubleSpeed_ = doubleSpeed;

	if (!haveRaw_)
	{
		haveRaw_ = true;
		lastRaw_ = cc;
	}
	else if (cc > lastRaw_)
	{
		now_ += (uint64_t)(cc - lastRaw_) << (doubleSpeed ? 0 : 1);
		lastRaw_ = cc;
	}
	else if (cc < lastRaw_)
	{
		/* gambatte rebases its cycle counter whenever it passes 2^31, so a step
		 * backwards is bookkeeping rather than time travel. The few cycles
		 * between the last reading and the rebase are given up; produced() puts
		 * the total back on the frontend's own count at the end of the chunk. */
		lastRaw_ = cc;
	}

	if (now_ < floor_)
		now_ = floor_;

	if (busy_ && now_ >= busyUntil_)
	{
		busy_ = false;
		published_ = false;
	}
}

void LinkSerial::wire(uint64_t tick, unsigned char type, unsigned char sb, unsigned char flags)
{
	unsigned char msg[LS_MSG_SIZE];

	/* Nothing may be sent before this machine has rendezvoused, because the
	 * rendezvous is what anchors the origin the bus measures a tick from. A
	 * message sent between a bus rebuild and the next one carries an offset from
	 * an origin that has been thrown away, and lands in the peer's far future
	 * where its clock never reaches it. Asking to advance to where this machine
	 * already stands is granted at once, so this costs a lock and never a wait. */
	if (!anchored_)
	{
		grant_ = link_->advance(handle_, now_, now_ + horizon_, now_, 0);
		anchored_ = true;
	}

	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	msg[1] = (unsigned char)selfId_;
	msg[2] = sb;
	msg[3] = flags;
	link_->send(handle_, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

void LinkSerial::publish(unsigned char sb, unsigned char sc)
{
	unsigned char flags = 0;
	unsigned char out;

	if (!attached_)
		return;

	/* Armed means the guest has set the transfer bit and is waiting to be
	 * clocked. A machine in the middle of a transfer is NOT armed even though
	 * that bit is still set: the peer's byte is shifted into SB as the transfer
	 * runs, so what the register holds part way through is half of each byte and
	 * belongs to nobody. Saying so keeps a peer that clocks during it from
	 * reading the wreckage.
	 *
	 * The transfer SPEED is deliberately not published. SC bit 1 selects the Game
	 * Boy Color's faster clock, but an original Game Boy reads it back as 1
	 * whatever was written -- gambatte ORs 0x7E into every SC write, as the
	 * hardware's unused bits do -- so a state carrying it would call every DMG on
	 * the wire fast. Speed belongs to whichever machine clocks the transfer, and
	 * that is what LS_CLOCK carries. */
	if ((sc & 0x80) && !busy_)
		flags |= LS_FLAG_ARMED;

	out = (flags & LS_FLAG_ARMED) ? sb : 0xFF;

	if (published_ && out == lastSb_ && flags == lastFlags_)
		return;

	lastSb_ = out;
	lastFlags_ = flags;
	published_ = true;

	/* Stamped a horizon ahead, which is the promise this machine published to
	 * the bus and has to keep. A peer reading it at that tick is reading a
	 * register this one settled a horizon ago -- which is exactly the delay a
	 * cable's own length would not give you, and is the price of letting two
	 * machines run without asking each other's permission every cycle. */
	wire(now_ + horizon_, LS_STATE, out, flags);
}

void LinkSerial::queue(const Pending &msg)
{
	if (pendingCount_ >= PENDING_MAX)
	{
		/* Nothing sane fills this: a peer publishes on change, and it cannot change
		 * more than a handful of times inside one horizon, because it can never be
		 * more than one ahead of this machine.
		 *
		 * So this drops the message and says so, rather than making room by acting
		 * on an older one early. Acting early would change what the guest reads at a
		 * moment decided by how many messages happened to have arrived, which is
		 * wall-clock luck, and that is the one thing that must not reach a guest two
		 * netplay peers are both replaying. If this ever prints, the reasoning above
		 * is wrong and that is worth knowing. */
		gambatte_log(RETRO_LOG_WARN,
			"Link cable: inbox full, dropping a message the horizon should have made"
			" impossible\n");
		return;
	}
	pending_[pendingCount_++] = msg;
}

void LinkSerial::apply(const Pending &msg)
{
	switch (msg.type)
	{
	case LS_STATE:
		peerSb_ = msg.sb;
		peerArmed_ = (msg.flags & LS_FLAG_ARMED) != 0;
		break;
	case LS_CLOCK:
		clockReady_ = true;
		clockAt_ = msg.tick;
		clockSb_ = msg.sb;
		clockFast_ = (msg.flags & LS_FLAG_FAST) != 0;
		break;
	default:
		break;
	}
}

void LinkSerial::applyDue()
{
	while (pendingCount_ > 0 && pending_[0].tick <= now_)
	{
		apply(pending_[0]);
		memmove(&pending_[0], &pending_[1], sizeof(Pending) * (pendingCount_ - 1));
		--pendingCount_;
	}

	/* A clock nobody answered. The guest was not armed when the peer drove the
	 * line, so this machine took no part in that transfer and must not take part
	 * in it later either: leaving it standing would hand the byte to whatever
	 * the game arms NEXT, one transfer out of step, and a link protocol that
	 * slips a byte never recovers. */
	if (clockReady_ && now_ > clockAt_ + transferTicks(clockFast_))
		clockReady_ = false;
}

void LinkSerial::pump()
{
	unsigned char buf[LS_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(buf);

	if (!attached_)
		return;

	while (link_->recv(handle_, &tick, &from, buf, &len))
	{
		if (len == LS_MSG_SIZE)
		{
			Pending msg;
			msg.tick = tick;
			msg.type = buf[0];
			msg.sb = buf[2];
			msg.flags = buf[3];
			queue(msg);
		}
		len = sizeof(buf);
	}
}

void LinkSerial::refreshPeers()
{
	unsigned was = peers_;
	unsigned count = 0;
	int id;

	if (!attached_)
		return;

	id = link_->peers(handle_, &count);
	if (id < 0)
	{
		selfId_ = 0;
		peers_ = 0;
	}
	else if (count > 2)
	{
		/* The bus is protocol-agnostic and will happily join five machines; a
		 * Game Boy link cable joins two. The four-player adapter existed, but
		 * gambatte does not emulate it, so carrying a third machine's bytes
		 * would be inventing a transfer rather than reproducing one. */
		gambatte_log(RETRO_LOG_WARN,
		             "Link cable: %u machines on one wire; a Game Boy link cable carries 2\n", count);
		selfId_ = 0;
		peers_ = 0;
	}
	else
	{
		selfId_ = id;
		peers_ = count;
	}

	if (peers_ != was)
	{
		/* Forget the other end, and say everything again.
		 *
		 * What was on the wire a moment ago is not evidence about what is on it
		 * now: a lead that has moved may be joining two different machines, and
		 * even the same one has been running unwatched in the meantime. Keeping
		 * the last state read would let a machine clock a byte out of a register
		 * it has not been told the contents of since before the cable moved.
		 *
		 * And a machine cabled in after the last change would otherwise never hear
		 * a state at all, leaving both ends waiting to be told the other was
		 * there. Anything already in flight belonged to the old cable and goes
		 * with it. */
		peerSb_ = 0xFF;
		peerArmed_ = false;
		published_ = false;
		anchored_ = false;
		reannounceUntil_ = now_ + LINK_REANNOUNCE_GRAINS * grain_;
		clockReady_ = false;
		pendingCount_ = 0;
		gambatte_log(RETRO_LOG_INFO, "Link cable: %u machine(s) on the wire, this one is %d\n",
		             peers_, selfId_);
	}
}

unsigned LinkSerial::advance(unsigned wanted)
{
	uint64_t headroom;
	unsigned budget;

	if (!attached_)
		return wanted;

	grain_ = fastSeen_ ? LINK_GRAIN_FAST : LINK_GRAIN_NORMAL;
	horizon_ = grain_;

	refreshPeers();

	/* Publish before reading. A peer parked on this machine's horizon cannot
	 * move until it has been told the horizon moved, and it may be sitting on
	 * the very message this machine is about to want. */
	uint32_t wakeFlags = RETRO_LINK_WAKE_NONE;
	grant_ = link_->advance(handle_, now_, now_ + horizon_, now_ + grain_, &wakeFlags);
	anchored_ = true;
	pump();
	applyDue();

	/* Say it again for a while after the cable moves. A state published while the
	 * peer was still off its timeline was dropped on the way, and this machine's
	 * serial registers may not change again for minutes -- a Game Boy waiting to
	 * be clocked writes SB and SC once and then does nothing at all -- so there
	 * is no second chance coming on its own. */
	if (peers_ >= 2 && now_ < reannounceUntil_)
		published_ = false;

	if (grant_ == RETRO_LINK_UNBOUNDED)
		return wanted;
	if (wakeFlags != RETRO_LINK_WAKE_NONE && grant_ <= now_)
		return 1;

	headroom = (grant_ > now_) ? (grant_ - now_) : 0;
	budget = (unsigned)(headroom / 4);

	/* Never zero: a chunk that produces nothing would spin retro_run without
	 * the machine ever reaching the tick it has been granted. */
	if (budget < 8)
		budget = 8;
	if (budget > wanted)
		budget = wanted;
	return budget;
}

void LinkSerial::produced(unsigned samples)
{
	if (!attached_)
		return;

	floor_ += (uint64_t)samples * 4;
	if (now_ < floor_)
		now_ = floor_;

	/* The chunk's end is now the anchor. Re-reading the cycle counter from here
	 * rather than carrying the old reading forward is what stops the tail of
	 * every chunk -- everything after its last event -- being counted twice. */
	haveRaw_ = false;

	if (busy_ && now_ >= busyUntil_)
	{
		busy_ = false;
		published_ = false;
	}
}

void LinkSerial::tick(unsigned long cc, bool doubleSpeed, unsigned char sb, unsigned char sc)
{
	if (!attached_)
		return;

	note(cc, doubleSpeed);
	applyDue();
	publish(sb, sc);
}

bool LinkSerial::check(unsigned char out, unsigned char &in, bool &fastCgb)
{
	/* `out` is this machine's own serial register, which the network version
	 * writes back down the socket here. Nothing does with it: the peer read this
	 * machine's register out of the state it published, a horizon ago, because
	 * on a real cable the byte is already sitting in the shift register when the
	 * clock arrives -- there is nothing to ask for and nobody to ask. */
	(void)out;

	if (!attached_ || peers_ < 2)
		return false;

	pump();
	applyDue();

	if (!clockReady_ || now_ < clockAt_)
		return false;

	clockReady_ = false;
	in = clockSb_;
	fastCgb = clockFast_;
	fastSeen_ = clockFast_;

	busy_ = true;
	busyUntil_ = now_ + transferTicks(clockFast_);
	published_ = false;
	return true;
}

unsigned char LinkSerial::send(unsigned char data, bool fastCgb)
{
	if (!attached_ || peers_ < 2)
		return 0xFF;

	pump();
	applyDue();
	fastSeen_ = fastCgb;

	/* Announced a horizon out, not at this instant. The peer may have run as far
	 * as this machine's published horizon and no further, so a transfer stamped
	 * there is one it is guaranteed not to have passed -- which is the whole
	 * reason the promise exists. It also means the peer's half of the transfer
	 * starts a horizon after this machine's; the two interrupts land that far
	 * apart, and no link protocol notices, because on real hardware the gap
	 * between one byte and the next is measured in milliseconds. */
	wire(now_ + horizon_, LS_CLOCK, data, fastCgb ? LS_FLAG_FAST : 0);

	busy_ = true;
	busyUntil_ = now_ + transferTicks(fastCgb);
	published_ = false;


	/* What the other end had in its register. An unarmed peer reads back as the
	 * 0xFF of an open line, which is what a Game Boy clocking a cable with
	 * nothing listening on it gets, and what the game is written to expect. */
	return peerArmed_ ? peerSb_ : 0xFF;
}
