/*

Copyright (c) 2020 Sergej Schumilo, Cornelius Aschermann

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Note:
This Intel PT software decoder is partially inspired and based on Andi
Kleen's fastdecode.c (simple-pt). m
See: https://github.com/andikleen/simple-pt/blob/master/fastdecode.c

 * Simple PT dumper
 *
 * Copyright (c) 2015, Intel Corporation
 * Author: Andi Kleen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#define _GNU_SOURCE

#include "core.h"
#include "libxdc.h"
#include "decoder.h"


#define BENCHMARK 				1


#define PT_TRACE_END			0x55

#define PT_PKT_GENERIC_LEN		2
#define PT_PKT_GENERIC_BYTE0	0x02

#define PT_PKT_LTNT_LEN			8
#define PT_PKT_LTNT_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_LTNT_BYTE1		0xA3

#define PT_PKT_PIP_LEN			8
#define PT_PKT_PIP_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_PIP_BYTE1		0x43

#define PT_PKT_CBR_LEN			4
#define PT_PKT_CBR_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_CBR_BYTE1		0x03

#define PT_PKT_OVF_LEN			2
#define PT_PKT_OVF_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_OVF_BYTE1		0xF3

#define PT_PKT_PSB_LEN			16
#define PT_PKT_PSB_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_PSB_BYTE1		0x82

#define PT_PKT_PSBEND_LEN		2
#define PT_PKT_PSBEND_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_PSBEND_BYTE1		0x23

#define PT_PKT_MNT_LEN			11
#define PT_PKT_MNT_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_MNT_BYTE1		0xC3
#define PT_PKT_MNT_BYTE2		0x88

#define PT_PKT_TMA_LEN			7
#define PT_PKT_TMA_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_TMA_BYTE1		0x73

#define PT_PKT_VMCS_LEN			7
#define PT_PKT_VMCS_BYTE0		PT_PKT_GENERIC_BYTE0
#define PT_PKT_VMCS_BYTE1		0xC8

#define	PT_PKT_TS_LEN			2
#define PT_PKT_TS_BYTE0			PT_PKT_GENERIC_BYTE0
#define PT_PKT_TS_BYTE1			0x83

#define PT_PKT_MODE_LEN			2
#define PT_PKT_MODE_BYTE0		0x99

#define PT_PKT_TIP_LEN			8
#define PT_PKT_TIP_SHIFT		5
#define PT_PKT_TIP_MASK			0x1F
#define PT_PKT_TIP_BYTE0		0x0D
#define PT_PKT_TIP_PGE_BYTE0	0x11
#define PT_PKT_TIP_PGD_BYTE0	0x01
#define PT_PKT_TIP_FUP_BYTE0	0x1D


#define TIP_VALUE_0				(0x0<<5)
#define TIP_VALUE_1				(0x1<<5)
#define TIP_VALUE_2				(0x2<<5)
#define TIP_VALUE_3				(0x3<<5)
#define TIP_VALUE_4				(0x4<<5)
#define TIP_VALUE_5				(0x5<<5)
#define TIP_VALUE_6				(0x6<<5)
#define TIP_VALUE_7				(0x7<<5)

//#define DEBUG

static decoder_state_machine_t* decoder_statemachine_new(void);
static void decoder_statemachine_reset(decoder_state_machine_t* self);

static uint8_t psb[16] = {
	0x02, 0x82, 0x02, 0x82, 0x02, 0x82, 0x02, 0x82,
	0x02, 0x82, 0x02, 0x82, 0x02, 0x82, 0x02, 0x82
};

#ifdef DECODER_LOG
static void flush_log(decoder_t* self){
	self->log.tnt64 = 0;
	self->log.tnt8 = 0;
	self->log.pip = 0;
	self->log.cbr = 0;
	self->log.ts = 0;
	self->log.ovf = 0;
	self->log.psbc = 0;
	self->log.psbend = 0;
	self->log.mnt = 0;
	self->log.tma = 0;
	self->log.vmcs = 0;
	self->log.pad = 0;
	self->log.tip = 0;
	self->log.tip_pge = 0;
	self->log.tip_pgd = 0;
	self->log.tip_fup = 0;
	self->log.mode = 0;
}
#endif

decoder_t* pt_decoder_init(){
	decoder_t* res = calloc(1, sizeof(decoder_t));

#ifdef DECODER_LOG
	flush_log(res);
#endif

	res->tnt_cache_state = tnt_cache_init();
		/* ToDo: Free! */
	res->decoder_state = decoder_statemachine_new();
	res->decoder_state_result = malloc(sizeof(should_disasm_t));
	res->decoder_state_result->start = 0;
	res->decoder_state_result->valid = 0;
	res->decoder_state_result->valid = false;
	res->mode = mode_64;

	return res;
}

void pt_decoder_destroy(decoder_t* self){
    if ( !self )
        return;

	if(self->tnt_cache_state){
		//destroy_disassembler(self->disassembler_state);
		tnt_cache_destroy(self->tnt_cache_state);
		self->tnt_cache_state = NULL;
	}
	free(self->decoder_state_result);
	free(self->decoder_state);
	free(self);
}

void pt_decoder_flush(decoder_t* self){
	self->ovp_state = false;
	self->last_tip = 0;
	self->last_fup_src = 0;
	self->fup_bind_pending = false;
#ifdef DECODER_LOG
	flush_log(self);
#endif

	tnt_cache_flush(self->tnt_cache_state);
	decoder_statemachine_reset(self->decoder_state);
	self->decoder_state_result->start = 0;
	self->decoder_state_result->valid = 0;
	self->decoder_state_result->valid = false;
}

uint64_t pt_decoder_get_page_fault_addr(decoder_t* self){
	return self->page_fault_addr;
}

static inline void _set_disasm(should_disasm_t* self, uint64_t from, uint64_t to){
	self->valid = true;
	self->start = from;
	self->end = to;
}

static decoder_state_machine_t* decoder_statemachine_new(void){
	decoder_state_machine_t * res = (decoder_state_machine_t*)malloc(sizeof(decoder_state_machine_t));
	res->state = TraceDisabled;
	res->last_ip = 0;
	return res;
}

static void decoder_statemachine_reset(decoder_state_machine_t* self){
	self->state = TraceDisabled;
	self->last_ip = 0;
}

static inline void decoder_handle_tip(decoder_state_machine_t *self, uint64_t addr, should_disasm_t *res){
	res->valid= false;
	switch(self->state){
		case TraceDisabled:
			_set_disasm(res, addr, 0);
			self->state = TraceEnabledWithLastIP;
			self->last_ip = addr;
			//assert(false);
			break;
		case TraceEnabledWithLastIP:
			_set_disasm(res, self->last_ip, 0);
			self->state = TraceEnabledWithLastIP;
			self->last_ip = addr;
			break;
		case TraceEnabledWOLastIP:
			self->state = TraceEnabledWithLastIP;
			self->last_ip = addr;
			break;
	}
}

static inline void decoder_handle_pgd(decoder_state_machine_t *self, uint64_t addr, should_disasm_t *res){
	res->valid= false;
	switch(self->state){
		case TraceDisabled:
			//assert(false);
			break;
		case TraceEnabledWithLastIP:
			_set_disasm(res, self->last_ip, addr);
			self->state = TraceDisabled;
			self->last_ip = 0;
			break;
		case TraceEnabledWOLastIP:
			self->state = TraceDisabled;
			break;
	}
}

static inline void decoder_handle_pge(decoder_state_machine_t *self, uint64_t addr, should_disasm_t *res){
	res->valid= false;
	switch(self->state){
		case TraceDisabled:
			self->state = TraceEnabledWithLastIP;
			self->last_ip = addr;
			break;
		case TraceEnabledWithLastIP:
			//assert(false);
			break;
		case TraceEnabledWOLastIP:
			self->state = TraceEnabledWithLastIP;
			self->last_ip = addr;
			break;
	}
}


static inline void decoder_handle_fup(decoder_state_machine_t *self, uint64_t fup_src, should_disasm_t *res){
	//assert(self->state);
	res->valid= false;
	switch(self->state){
		case TraceDisabled:
			self->state = TraceDisabled;
			break;
		case TraceEnabledWithLastIP:
			_set_disasm(res, self->last_ip, fup_src);
			//self->state = TraceEnabledWOLastIP;
			//self->last_ip = 0;
			self->last_ip = fup_src;
		      break;
		case TraceEnabledWOLastIP:
			//assert(false);
			break;
	}
}

static inline uint64_t get_ip_val(decoder_t* self, uint8_t **pp){
    const uint8_t type = (*(*pp)++ >> 5);
    uint64_t aligned_last_ip, aligned_pp;
    memcpy(&aligned_pp, *pp, sizeof(uint64_t));
    memcpy(&aligned_last_ip, &self->last_tip, sizeof(uint64_t));

    if (unlikely(type == 0)) {
        return 0;
    }

    const uint8_t new_bits = 0xFF40FF30302010FFull >> (type * 8);
    if (unlikely(type == 3)) {
        aligned_last_ip = (int64_t)(aligned_pp << 16) >> 16;
    } else {
        const uint8_t old_bits = sizeof(aligned_last_ip) * 8 - new_bits;   // always less than 64
        const uint64_t new_mask = (~0ull) >> old_bits;
        const uint64_t old_mask = ~new_mask;
        aligned_last_ip = (aligned_last_ip & old_mask) | (aligned_pp & new_mask);
    }
    memcpy(&self->last_tip, &aligned_last_ip, sizeof(uint64_t));
    *pp += new_bits >> 3;

    if (unlikely(NULL != self->ip_callback))
        self->ip_callback(self->ip_callback_opaque, self->mode, aligned_last_ip);

    return aligned_last_ip;
}

static inline uint64_t get_val(uint8_t **pp, uint8_t len){
	uint8_t*p = *pp;
	uint64_t v = 0;
	uint8_t i;
	uint8_t shift = 0;

	for (i = 0; i < len; i++, shift += 8)
		v |= ((uint64_t)(*p++)) << shift;
	*pp = p;
	return v;
}



static inline void disasm(decoder_t* self){
	static uint64_t failed_page = 0;
	should_disasm_t* res = self->decoder_state_result;
	if(res->valid && (!is_empty_tnt_cache(self->tnt_cache_state))){
		LOGGER("disasm(%lx,%lx)\tTNT: %d\n", res->start, res->end, count_tnt(self->tnt_cache_state));
			if(unlikely(trace_disassembler(self->disassembler_state, res->start, res->end, self->tnt_cache_state, &failed_page, self->mode) == disas_page_fault)){
				self->page_fault_found = true;
				self->page_fault_addr = failed_page;
			}
	}
}


static void tip_handler(decoder_t* self, uint8_t** p){
	if(unlikely(self->fup_bind_pending)){
		self->fup_bind_pending = false;
		decoder_handle_fup(self->decoder_state, self->last_fup_src, self->decoder_state_result);
    self->last_fup_src = 0;
		disasm(self);
		if(unlikely(self->page_fault_found)){
			return;
		}
	}

	get_ip_val(self, p);

	LOGGER("TIP    \t%lx (TNT: %d)\n", self->last_tip, count_tnt(self->tnt_cache_state));
	decoder_handle_tip(self->decoder_state, self->last_tip, self->decoder_state_result);
	disasm(self);
#ifdef DECODER_LOG
	self->log.tip++;
#endif
}

static void tip_pge_handler(decoder_t* self, uint8_t** p){
	self->ovp_state = false;

	if(unlikely(self->fup_bind_pending)){
		self->fup_bind_pending = false;
		decoder_handle_fup(self->decoder_state, self->last_fup_src, self->decoder_state_result);
    	self->last_fup_src = 0;
		disasm(self);
		if(unlikely(self->page_fault_found)){
			return;
		}
	}

	get_ip_val(self, p);

	LOGGER("PGE    \t%lx (TNT: %d)\n", self->last_tip, count_tnt(self->tnt_cache_state));
	decoder_handle_pge(self->decoder_state, self->last_tip, self->decoder_state_result);
 	 assert(!self->decoder_state_result->valid);

	if(unlikely(self->disassembler_state->trace_mode)){
		self->disassembler_state->trace_edge_callback(self->disassembler_state->trace_edge_callback_opaque, self->mode, INIT_TRACE_IP, self->last_tip);
	}
#ifdef DECODER_LOG
	self->log.tip_pge++;
#endif
}

static void tip_pgd_handler(decoder_t* self, uint8_t** p){
	if(unlikely(self->fup_bind_pending)){
		self->fup_bind_pending = false;
		decoder_handle_fup(self->decoder_state, self->last_fup_src, self->decoder_state_result);
    	self->last_fup_src = 0;
		disasm(self);
		if(unlikely(self->page_fault_found)){
			return;
		}
	}

	get_ip_val(self, p);
	LOGGER("PGD    \t%lx (TNT: %d)\n", self->last_tip, count_tnt(self->tnt_cache_state));
	decoder_handle_pgd(self->decoder_state, self->last_tip, self->decoder_state_result);
	disasm(self);

	if(unlikely(self->disassembler_state->trace_mode)){
		if(self->disassembler_state->has_pending_indirect_branch){
			self->disassembler_state->has_pending_indirect_branch = false;
			self->disassembler_state->trace_edge_callback(self->disassembler_state->trace_edge_callback_opaque, self->mode, self->disassembler_state->pending_indirect_branch_src, self->last_tip);
		}
		self->disassembler_state->trace_edge_callback(self->disassembler_state->trace_edge_callback_opaque, self->mode, self->last_tip, INIT_TRACE_IP);
		//TODO, old code had:
		//redqueen_trace_register_transition(self, self->last_ip, ip);
		//redqueen_trace_register_transition(self, ip, INIT_TRACE_IP);
  }
#ifdef DECODER_LOG
	self->log.tip_pgd++;
#endif
}

static void tip_fup_handler(decoder_t* self, uint8_t** p){
//	printf("%s\n", __func__);
	if(self->ovp_state){
		self->decoder_state->state = TraceEnabledWithLastIP;
		self->decoder_state->last_ip = get_ip_val(self, p);

		LOGGER("FUP OVP   \t%lx (TNT: %d)\n", self->last_tip, count_tnt(self->tnt_cache_state));

		self->ovp_state = false;
		self->fup_bind_pending = false;

		return;
	}

	self->last_fup_src = get_ip_val(self, p);
	LOGGER("FUP    \t%lx (TNT: %d)\n", self->last_fup_src, count_tnt(self->tnt_cache_state));

	self->fup_bind_pending = true;
#ifdef DECODER_LOG
	self->log.tip_fup++;
#endif
}

static inline void pip_handler(decoder_t* self, uint8_t** p){
#ifdef SAMPLE_DECODED_DETAILED
	(*p) += PT_PKT_PIP_LEN-6;
	LOGGER("PIP\t%llx\n", (get_val(p, 6) >> 1) << 5);
#else
	//get_val(p, 6);
	(*p) += PT_PKT_PIP_LEN;
#endif
#ifdef DECODER_LOG
	self->log.pip++;
#endif
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
LIBXDC_HOT decoder_result_t decode_buffer(decoder_t* self, uint8_t* map, size_t len){

	bool pt_overflowed = false;
	self->page_fault_found = false;

	uint8_t *end = map + len;
	uint8_t *p = map;

#ifdef DECODER_LOG
	flush_log(self);
#endif

	p = memmem(p, end - p, psb, PT_PKT_PSB_LEN);
	if (!p) {
		p = end;
		goto handle_pt_exit;
	}

	while (true) {
		switch (p[0]) {
		case 0: goto handle_pt_pad;

		case 2: goto handle_pt_level_2;

		case 85: goto handle_pt_exit;

		case 5:
		case 9:
		case 21:
		case 37:
		case 41:
		case 53:
		case 57:
		case 69:
		case 73:
		case 101:
		case 105:
		case 117:
		case 121:
		case 133:
		case 137:
		case 149:
		case 165:
		case 169:
		case 181:
		case 185:
		case 197:
		case 201:
		case 213:
		case 217:
		case 229:
		case 233:
		case 245:
		case 249:
		case 255: goto handle_pt_error;

		default:
			if ((p[0] & 0x1F) == 0x01) {
				goto handle_pt_tip_pgd;
			}
			if ((p[0] & 0x03) == 0x03) {
				goto handle_pt_cyc;
			}
			if ((p[0] & 0x01) == 0) {
				goto handle_pt_tnt8;
			}
			if ((p[0] & 0x1F) == 0x1D) {
				goto handle_pt_tip_fup;
			}
			assert(false);
		}

		handle_pt_mode:
			/*
			// Code to test if TSX code has been executed inside the guest
			if ((((char*)(p))[1] & 0xE0) == 0x20){
				if ( (((char*)(p))[1] & 0x3))
					fprintf(stderr, "TSX FOUND %x\n", (((char*)(p))[1] & 0x3));
			}
			*/

			switch (p[1] >> 5) {
				case 0:
					switch (p[1] & 3) {
						case 0:
							self->mode = mode_16;
							break;
						case 1:
							self->mode = mode_64;
							break;
						case 2:
							self->mode = mode_32;
							break;
					}
				default:
					break;
			}

			p += PT_PKT_MODE_LEN;
			LOGGER("MODE\n");
			#ifdef DECODER_LOG
			self->log.mode++;
			#endif
			continue;
		handle_pt_tip:
			tip_handler(self, &p);
			if(unlikely(self->page_fault_found)){
				pt_decoder_flush(self);
				return decoder_page_fault;
			}
			continue;
		handle_pt_tip_pge:
			tip_pge_handler(self, &p);
			if(unlikely(self->page_fault_found)){
				pt_decoder_flush(self);
				return decoder_page_fault;
			}
			continue;
		handle_pt_tip_pgd:
			tip_pgd_handler(self, &p);
			if(unlikely(self->page_fault_found)){
				pt_decoder_flush(self);
				return decoder_page_fault;
			}
			continue;
		handle_pt_tip_fup:
			tip_fup_handler(self, &p);
			continue;
		handle_pt_pad:
			while(unlikely(!(*(++p)))){}
			//p++;
			#ifdef DECODER_LOG
			self->log.pad++;
			#endif
			continue;
		handle_pt_level_2:
			switch(p[1]){
				case 0x03:	/* CBR */
					p += PT_PKT_CBR_LEN;
					#ifdef DECODER_LOG
					self->log.cbr++;
					#endif
					continue;

				case 0x23:	/* PSBEND */
					p += PT_PKT_PSBEND_LEN;
					LOGGER("PSBEND\n");
					#ifdef DECODER_LOG
					self->log.psbend++;
					#endif
					continue;

				case 0x43:	/* PIP */
					pip_handler(self, &p);
					continue;

				case 0x82:	/* PSB */
					self->last_tip = 0;
					p += PT_PKT_PSB_LEN;
					LOGGER("PSB\n");
					#ifdef DECODER_LOG
					self->log.psbc++;
					#endif
					continue;

				case 0x83:	/* TS  */
					/*
					abort();
					fprintf(stderr, "\n\n===========> TS\n" );
					p += PT_PKT_TS_LEN;
					fprintf(stderr, "ERROR DETECTED...FLUSHING DISASSEMBLER!\n");
					*/
					return decoder_error;
					//continue;

				case 0xA3:	/* LTNT */
#ifdef LIBFUZZER
				return decoder_unkown_packet;
#endif
					LOGGER("LTNT\n");
					append_tnt_cache_ltnt(self->tnt_cache_state, (uint64_t)*p);
					p += PT_PKT_LTNT_LEN;
					#ifdef DECODER_LOG
					self->log.tnt64++;
					#endif
					continue;

				case 0xC8:	/* VMCS */
					LOGGER("VMCS\n");
					p += PT_PKT_VMCS_LEN;
					#ifdef DECODER_LOG
					self->log.vmcs++;
					#endif
					continue;

				case 0xF3:	/* OVF */
					LOGGER("OVERFLOW\n");
					p += PT_PKT_OVF_LEN;
					self->ovp_state = true;
					self->last_tip = 0;
					decoder_statemachine_reset(self->decoder_state);
					pt_overflowed = true;

					continue;

				case 0xC3:	/* MNT */
				case 0x73:	/* TMA */
				default:
					//printf("unkown packet (level2): %x\n", p[1]);
					pt_decoder_flush(self);
					return decoder_unkown_packet;
					//abort();

			}
		handle_pt_tnt8:
			LOGGER("TNT %x\n", *p);
			append_tnt_cache(self->tnt_cache_state, (uint64_t)(*p));
			p++;
			#ifdef DECODER_LOG
			self->log.tnt8++;
			#endif
			continue;
		handle_pt_mtc:
		handle_pt_tsc:
		handle_pt_error:
		handle_pt_cyc:
			//printf("unkown packet: %x\n", p[0]);
			pt_decoder_flush(self);
			return decoder_unkown_packet;
			//abort();
	}

	handle_pt_exit:

	if (count_tnt(self->tnt_cache_state) != 0){
#ifndef LIBFUZZER
		should_disasm_t* res = self->decoder_state_result;
		fprintf(stderr, "\nERR: \tTNT %d at position <0x%08lx,0x%08lx>\n",
				count_tnt(self->tnt_cache_state), res->start, res->end);
#endif
		pt_decoder_flush(self);
		return decoder_error;
	}

	if(self->disassembler_state->infinite_loop_found){
		abort();

		char* tmp;
		assert(asprintf(&tmp, "/tmp/loop_trace_%d_%d", getpid(), self->error_counter++) != -1);
		FILE* f = fopen(tmp, "wb");
							fwrite(map, len, 1, f);
							fclose(f);
		free(tmp);

		self->disassembler_state->infinite_loop_found = false;
	}

	pt_decoder_flush(self);

	if(pt_overflowed){
		return decoder_success_pt_overflow;
	}
	return decoder_success;
}
#pragma GCC diagnostic pop
