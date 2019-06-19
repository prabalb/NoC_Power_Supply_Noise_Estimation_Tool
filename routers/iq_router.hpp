// $Id: iq_router.hpp 5379 2013-01-10 03:58:09Z dub $

/*
 Copyright (c) 2007-2012, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _IQ_ROUTER_HPP_
#define _IQ_ROUTER_HPP_

#include <string>
#include <deque>
#include <queue>
#include <set>
#include <map>
#include <algorithm>

#include "router.hpp"
#include "routefunc.hpp"
#include "network.hpp"
#include "psn.hpp"

// Power Quantum Processes (Prabal)
#define PROCESS_INCOMING  0
#define PROCESS_SWALLOC   1
#define PROCESS_XBAR      2
#define PROCESS_FORWARD   3
#define PROCESS_STANDBY   4

#define SIM_WARMUP_TIME 1000

using namespace std;

class VC;
class Flit;
class Credit;
class Buffer;
class BufferState;
class Allocator;
class SwitchMonitor;
class BufferMonitor;
class Network;

class IQRouter : public Router {

  int _vcs;

  bool _vc_busy_when_full;
  bool _vc_prioritize_empty;
  bool _vc_shuffle_requests;

  bool _speculative;
  bool _spec_check_elig;
  bool _spec_check_cred;
  bool _spec_mask_by_reqs;
  
  bool _active;

  bool gen_trace;
  bool block_flits;
  int num_flits_to_receive;
  Network * Net;
  //mutable vector<bool> chStatus;
  PSN *psn;

  int _routing_delay;
  int _vc_alloc_delay;
  int _sw_alloc_delay;
  
  map<int, Flit *> _in_queue_flits;
  mutable map<int, vector<Flit *> > flitMap;

  deque<pair<int, pair<Credit *, int> > > _proc_credits;

  deque<pair<int, pair<int, int> > > _route_vcs;
  deque<pair<int, pair<pair<int, int>, int> > > _vc_alloc_vcs;  
  deque<pair<int, pair<pair<int, int>, int> > > _sw_hold_vcs;
  deque<pair<int, pair<pair<int, int>, int> > > _sw_alloc_vcs;

  deque<pair<int, pair<Flit *, pair<int, int> > > > _crossbar_flits;

  map<int, Credit *> _out_queue_credits;

  vector<Buffer *> _buf;
  vector<BufferState *> _next_buf;

  Allocator *_vc_allocator;
  Allocator *_sw_allocator;
  Allocator *_spec_sw_allocator;
  
  vector<int> _vc_rr_offset;
  vector<int> _sw_rr_offset;

  tRoutingFunction   _rf;

  int _output_buffer_size;
  vector<queue<Flit *> > _output_buffer;

  vector<queue<Credit *> > _credit_buffer;

  bool _hold_switch_for_packet;
  vector<int> _switch_hold_in;
  vector<int> _switch_hold_out;
  vector<int> _switch_hold_vc;

  bool _noq;
  vector<vector<int> > _noq_next_output_port;
  vector<vector<int> > _noq_next_vc_start;
  vector<vector<int> > _noq_next_vc_end;

#ifdef TRACK_FLOWS
  vector<vector<queue<int> > > _outstanding_classes;
#endif

  bool _ReceiveFlits( );
  bool _ReceiveCredits( );

  virtual void _InternalStep( );

  bool _SWAllocAddReq(int input, int vc, int output);

  void _InputQueuing( );

  void _RouteEvaluate( );
  void _VCAllocEvaluate( );
  void _SWHoldEvaluate( );
  void _SWAllocEvaluate( );
  void _SwitchEvaluate( );

  void _RouteUpdate( );
  void _VCAllocUpdate( );
  void _SWHoldUpdate( );
  void _SWAllocUpdate( );
  void _SwitchUpdate( );

  void _OutputQueuing( );

  void _SendFlits( );
  void _SendCredits( );
  
  void _UpdateNOQ(int input, int vc, Flit const * f);

  // ----------------------------------------
  //
  //   Router Power Modellingyes
  //
  // ----------------------------------------

  SwitchMonitor * _switchMonitor ;
  BufferMonitor * _bufferMonitor ;
  
public:

  IQRouter( Configuration const & config,
	    Module *parent, string const & name, int id,
	    int inputs, int outputs );
  
  virtual ~IQRouter( );
  
  virtual void AddOutputChannel(FlitChannel * channel, CreditChannel * backchannel);

  virtual void ReadInputs( );
  virtual void WriteOutputs( );
  
  void Display( ostream & os = cout ) const;

  virtual int GetUsedCredit(int o) const;
  virtual int GetBufferOccupancy(int i) const;
  virtual int GetXbarDemand(int o) const;
  virtual int GetNumBlockedFlits(int i) const;

#ifdef TRACK_BUFFERS
  virtual int GetUsedCreditForClass(int output, int cl) const;
  virtual int GetBufferOccupancyForClass(int input, int cl) const;
#endif

  virtual vector<int> UsedCredits() const;
  virtual vector<int> FreeCredits() const;
  virtual vector<int> MaxCredits() const;

  SwitchMonitor const * const GetSwitchMonitor() const {return _switchMonitor;}
  BufferMonitor const * const GetBufferMonitor() const {return _bufferMonitor;}

};

#endif

/* Group of routers -- Prabal Basu */

class Group {

private:
  std::map<int, std::vector<int> > groupToRouters;
  std::map<int, std::vector<bool> > groupSwitchMap;
  std::set<int> boundaryRouters;

private:
  Group() { initialize(); }
  void operator=(Group&);
  Group(const Group&);

public:
  static Group& getGroupObj() {
    static Group grp;
    return grp;
  }

  int getGroup(int id) {
    int radix = 8;
    int res = id / radix;
    int rem = id % radix;
    if(res == 0 || res == 1) {
      if(rem == 0 || rem == 1) return 0;
      if(rem == 2 || rem == 3) return 1;
      if(rem == 4 || rem == 5) return 2;
      if(rem == 6 || rem == 7) return 3;
    } else if(res == 2 || res == 3) {
      if(rem == 0 || rem == 1) return 4;
      if(rem == 2 || rem == 3) return 5;
      if(rem == 4 || rem == 5) return 6;
      if(rem == 6 || rem == 7) return 7;
    } else if(res == 4 || res == 5) {
      if(rem == 0 || rem == 1) return 8;
      if(rem == 2 || rem == 3) return 9;
      if(rem == 4 || rem == 5) return 10;
      if(rem == 6 || rem == 7) return 11;
    } else if(res == 6 || res == 7) {
      if(rem == 0 || rem == 1) return 12;
      if(rem == 2 || rem == 3) return 13;
      if(rem == 4 || rem == 5) return 14;
      if(rem == 6 || rem == 7) return 15;
    }
    return -1; // control should never reach here
  }

  bool receiveMaxFlits(int id) {
    int grp = getGroup(id);
    assert(grp >= 0 && grp <= 15);
    std::vector<int> rtrs = groupToRouters[grp];
    size_t i = 0;
    for(; i < 4; i++) {
      if(rtrs[i] == id) break;
    }
    if(groupSwitchMap[grp][i] == true) return true;
    return false;
  }

  int getNumFlitsToReceive(int id, Network *Net, int time) {
    if(receiveMaxFlits(id)) return 5;
    if(boundaryRouters.find(id) != boundaryRouters.end()) return 4;
    if(time % 10 == 0) return 3;
    return 4;

    int grp = getGroup(id);
    assert(grp >= 0 && grp <= 15);
    assert(Net != NULL);
    const vector<Router *>& rtrs  = Net->GetRouters();
    std::vector<int> grpRouters = groupToRouters[grp];
    std::vector<int> congVec (4,0);
    for(size_t i = 0; i < grpRouters.size(); i++) {
      for(int j = 0; j < 5; j++) {
        congVec[i] += rtrs[grpRouters[i]]->GetBufferOccupancy(j);
      }
    }
    int pos = 0;
    for(size_t i = 0; i < grpRouters.size(); i++) {
      if(grpRouters[i] == id) {
        pos = i;
        break;
      }
    }
    std::vector<int> congVec_copy = congVec;
    std::sort(congVec.begin(), congVec.end());
    int cong_rank = 0;
    for(size_t i = 0; i < congVec.size(); i++) {
      if(congVec_copy[pos] == congVec[i]) {
        cong_rank = i;
        break;
      }
    }
    if(cong_rank == 0) {
      return 5;
    } else {
      if(time % 10 == 0) return 3;
      return 4;
    }
  }

  void update() {
    for(int i = 0; i < 16; i++) {
      std::vector<bool> swStatus = groupSwitchMap[i];
      std::vector<bool> swStatus_copy = swStatus;
      swStatus[0] = swStatus_copy[3];
      swStatus[1] = swStatus_copy[0];
      swStatus[2] = swStatus_copy[1];
      swStatus[3] = swStatus_copy[2];
      groupSwitchMap[i] = swStatus;
    }
  }

  void initialize() {

    groupToRouters[0].push_back(0);
    groupToRouters[0].push_back(1);
    groupToRouters[0].push_back(9);
    groupToRouters[0].push_back(8);

    groupToRouters[1].push_back(2);
    groupToRouters[1].push_back(3);
    groupToRouters[1].push_back(11);
    groupToRouters[1].push_back(10);

    groupToRouters[2].push_back(4);
    groupToRouters[2].push_back(5);
    groupToRouters[2].push_back(13);
    groupToRouters[2].push_back(12);

    groupToRouters[3].push_back(6);
    groupToRouters[3].push_back(7);
    groupToRouters[3].push_back(15);
    groupToRouters[3].push_back(14);

    groupToRouters[4].push_back(16);
    groupToRouters[4].push_back(17);
    groupToRouters[4].push_back(25);
    groupToRouters[4].push_back(24);

    groupToRouters[5].push_back(18);
    groupToRouters[5].push_back(19);
    groupToRouters[5].push_back(27);
    groupToRouters[5].push_back(26);

    groupToRouters[6].push_back(20);
    groupToRouters[6].push_back(21);
    groupToRouters[6].push_back(29);
    groupToRouters[6].push_back(28);

    groupToRouters[7].push_back(22);
    groupToRouters[7].push_back(23);
    groupToRouters[7].push_back(31);
    groupToRouters[7].push_back(30);

    groupToRouters[8].push_back(32);
    groupToRouters[8].push_back(33);
    groupToRouters[8].push_back(41);
    groupToRouters[8].push_back(40);

    groupToRouters[9].push_back(34);
    groupToRouters[9].push_back(35);
    groupToRouters[9].push_back(43);
    groupToRouters[9].push_back(42);

    groupToRouters[10].push_back(36);
    groupToRouters[10].push_back(37);
    groupToRouters[10].push_back(45);
    groupToRouters[10].push_back(44);

    groupToRouters[11].push_back(38);
    groupToRouters[11].push_back(39);
    groupToRouters[11].push_back(47);
    groupToRouters[11].push_back(46);

    groupToRouters[12].push_back(48);
    groupToRouters[12].push_back(49);
    groupToRouters[12].push_back(57);
    groupToRouters[12].push_back(56);

    groupToRouters[13].push_back(50);
    groupToRouters[13].push_back(51);
    groupToRouters[13].push_back(59);
    groupToRouters[13].push_back(58);

    groupToRouters[14].push_back(52);
    groupToRouters[14].push_back(53);
    groupToRouters[14].push_back(61);
    groupToRouters[14].push_back(60);

    groupToRouters[15].push_back(54);
    groupToRouters[15].push_back(55);
    groupToRouters[15].push_back(63);
    groupToRouters[15].push_back(62);

    for(int i = 0; i < 16; i++) {
      groupSwitchMap[i].push_back(true);
      groupSwitchMap[i].push_back(false);
      groupSwitchMap[i].push_back(false);
      groupSwitchMap[i].push_back(false);
    }
    int brtrs[] = {0,1,2,3,4,5,6,7,8,15,16,23,24,31,32,39,40,47,48,55,56,57,58,59,60,61,62,63};
    set<int> bRtrsSet(brtrs, brtrs+28);
    boundaryRouters = bRtrsSet;
  }
};

