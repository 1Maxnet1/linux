// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/* Copyright (c) 2020 Marvell International Ltd. All rights reserved */

#include "prestera.h"
#include "prestera_acl.h"
#include "prestera_flow.h"
#include "prestera_flower.h"

static int prestera_flower_parse_actions(struct prestera_flow_block *block,
					 struct prestera_acl_rule *rule,
					 struct flow_action *flow_action,
					 struct netlink_ext_ack *extack)
{
	const struct flow_action_entry *act;
	int i;

	/* whole struct (rule->re_arg) must be initialized with 0 */
	if (!flow_action_has_entries(flow_action))
		return 0;

	flow_action_for_each(i, act, flow_action) {
		switch (act->id) {
		case FLOW_ACTION_ACCEPT:
			if (rule->re_arg.accept.valid)
				return -EEXIST;

			rule->re_arg.accept.valid = 1;
			break;
		case FLOW_ACTION_DROP:
			if (rule->re_arg.drop.valid)
				return -EEXIST;

			rule->re_arg.drop.valid = 1;
			break;
		case FLOW_ACTION_TRAP:
			if (rule->re_arg.trap.valid)
				return -EEXIST;

			rule->re_arg.trap.valid = 1;
			break;
		default:
			NL_SET_ERR_MSG_MOD(extack, "Unsupported action");
			pr_err("Unsupported action\n");
			return -EOPNOTSUPP;
		}
	}

	return 0;
}

static int prestera_flower_parse_meta(struct prestera_acl_rule *rule,
				      struct flow_cls_offload *f,
				      struct prestera_flow_block *block)
{	struct flow_rule *f_rule = flow_cls_offload_flow_rule(f);
	struct prestera_acl_match *r_match = &rule->re_key.match;
	struct prestera_port *port;
	struct net_device *ingress_dev;
	struct flow_match_meta match;
	__be16 key, mask;

	flow_rule_match_meta(f_rule, &match);
	if (match.mask->ingress_ifindex != 0xFFFFFFFF) {
		NL_SET_ERR_MSG_MOD(f->common.extack,
				   "Unsupported ingress ifindex mask");
		return -EINVAL;
	}

	ingress_dev = __dev_get_by_index(block->net,
					 match.key->ingress_ifindex);
	if (!ingress_dev) {
		NL_SET_ERR_MSG_MOD(f->common.extack,
				   "Can't find specified ingress port to match on");
		return -EINVAL;
	}

	if (!prestera_netdev_check(ingress_dev)) {
		NL_SET_ERR_MSG_MOD(f->common.extack,
				   "Can't match on switchdev ingress port");
		return -EINVAL;
	}
	port = netdev_priv(ingress_dev);

	mask = htons(0x1FFF);
	key = htons(port->hw_id);
	rule_match_set(r_match->key, SYS_PORT, key);
	rule_match_set(r_match->mask, SYS_PORT, mask);

	mask = htons(0x1FF);
	key = htons(port->dev_id);
	rule_match_set(r_match->key, SYS_DEV, key);
	rule_match_set(r_match->mask, SYS_DEV, mask);

	return 0;

}

static int prestera_flower_parse(struct prestera_flow_block *block,
				 struct prestera_acl_rule *rule,
				 struct flow_cls_offload *f)
{	struct flow_rule *f_rule = flow_cls_offload_flow_rule(f);
	struct flow_dissector *dissector = f_rule->match.dissector;
	struct prestera_acl_match *r_match = &rule->re_key.match;
	__be16 n_proto_mask = 0;
	__be16 n_proto_key = 0;
	u16 addr_type = 0;
	u8 ip_proto = 0;
	int err;

	if (dissector->used_keys &
	    ~(BIT(FLOW_DISSECTOR_KEY_META) |
	      BIT(FLOW_DISSECTOR_KEY_CONTROL) |
	      BIT(FLOW_DISSECTOR_KEY_BASIC) |
	      BIT(FLOW_DISSECTOR_KEY_ETH_ADDRS) |
	      BIT(FLOW_DISSECTOR_KEY_IPV4_ADDRS) |
	      BIT(FLOW_DISSECTOR_KEY_IPV6_ADDRS) |
	      BIT(FLOW_DISSECTOR_KEY_ICMP) |
	      BIT(FLOW_DISSECTOR_KEY_PORTS) |
	      BIT(FLOW_DISSECTOR_KEY_VLAN))) {
		NL_SET_ERR_MSG_MOD(f->common.extack, "Unsupported key");
		return -EOPNOTSUPP;
	}

	prestera_acl_rule_priority_set(rule, f->common.prio);

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_META)) {
		err = prestera_flower_parse_meta(rule, f, block);
		if (err)
			return err;
	}

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_CONTROL)) {
		struct flow_match_control match;

		flow_rule_match_control(f_rule, &match);
		addr_type = match.key->addr_type;
	}

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_BASIC)) {
		struct flow_match_basic match;

		flow_rule_match_basic(f_rule, &match);
		n_proto_key = match.key->n_proto;
		n_proto_mask = match.mask->n_proto;

		if (ntohs(match.key->n_proto) == ETH_P_ALL) {
			n_proto_key = 0;
			n_proto_mask = 0;
		}

		rule_match_set(r_match->key, ETH_TYPE, n_proto_key);
		rule_match_set(r_match->mask, ETH_TYPE, n_proto_mask);

		rule_match_set(r_match->key, IP_PROTO, match.key->ip_proto);
		rule_match_set(r_match->mask, IP_PROTO, match.mask->ip_proto);
		ip_proto = match.key->ip_proto;
	}

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		struct flow_match_eth_addrs match;

		flow_rule_match_eth_addrs(f_rule, &match);

		/* DA key, mask */
		rule_match_set_n(r_match->key,
				 ETH_DMAC_0, &match.key->dst[0], 4);
		rule_match_set_n(r_match->key,
				 ETH_DMAC_1, &match.key->dst[4], 2);

		rule_match_set_n(r_match->mask,
				 ETH_DMAC_0, &match.mask->dst[0], 4);
		rule_match_set_n(r_match->mask,
				 ETH_DMAC_1, &match.mask->dst[4], 2);

		/* SA key, mask */
		rule_match_set_n(r_match->key,
				 ETH_SMAC_0, &match.key->src[0], 4);
		rule_match_set_n(r_match->key,
				 ETH_SMAC_1, &match.key->src[4], 2);

		rule_match_set_n(r_match->mask,
				 ETH_SMAC_0, &match.mask->src[0], 4);
		rule_match_set_n(r_match->mask,
				 ETH_SMAC_1, &match.mask->src[4], 2);
	}

	if (addr_type == FLOW_DISSECTOR_KEY_IPV4_ADDRS) {
		struct flow_match_ipv4_addrs match;

		flow_rule_match_ipv4_addrs(f_rule, &match);

		rule_match_set(r_match->key, IP_SRC, match.key->src);
		rule_match_set(r_match->mask, IP_SRC, match.mask->src);

		rule_match_set(r_match->key, IP_DST, match.key->dst);
		rule_match_set(r_match->mask, IP_DST, match.mask->dst);
	}

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_PORTS)) {
		struct flow_match_ports match;

		if (ip_proto != IPPROTO_TCP && ip_proto != IPPROTO_UDP) {
			NL_SET_ERR_MSG_MOD
			    (f->common.extack,
			     "Only UDP and TCP keys are supported");
			return -EINVAL;
		}

		flow_rule_match_ports(f_rule, &match);

		rule_match_set(r_match->key, L4_PORT_SRC, match.key->src);
		rule_match_set(r_match->mask, L4_PORT_SRC, match.mask->src);

		rule_match_set(r_match->key, L4_PORT_DST, match.key->dst);
		rule_match_set(r_match->mask, L4_PORT_DST, match.mask->dst);
	}

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_VLAN)) {
		struct flow_match_vlan match;

		flow_rule_match_vlan(f_rule, &match);

		if (match.mask->vlan_id != 0) {
			__be16 key = cpu_to_be16(match.key->vlan_id);
			__be16 mask = cpu_to_be16(match.mask->vlan_id);

			rule_match_set(r_match->key, VLAN_ID, key);
			rule_match_set(r_match->mask, VLAN_ID, mask);
		}

		rule_match_set(r_match->key, VLAN_TPID, match.key->vlan_tpid);
		rule_match_set(r_match->mask, VLAN_TPID, match.mask->vlan_tpid);
	}

	if (flow_rule_match_key(f_rule, FLOW_DISSECTOR_KEY_ICMP)) {
		struct flow_match_icmp match;

		flow_rule_match_icmp(f_rule, &match);

		rule_match_set(r_match->key, ICMP_TYPE, match.key->type);
		rule_match_set(r_match->mask, ICMP_TYPE, match.mask->type);

		rule_match_set(r_match->key, ICMP_CODE, match.key->code);
		rule_match_set(r_match->mask, ICMP_CODE, match.mask->code);
	}

	return prestera_flower_parse_actions(block, rule, &f->rule->action,
					     f->common.extack);
}

int prestera_flower_replace(struct prestera_flow_block *block,
			    struct flow_cls_offload *f)
{
	struct prestera_acl_ruleset *ruleset;
	struct prestera_acl *acl = block->sw->acl;
	struct prestera_acl_rule *rule;
	int err;

	ruleset = prestera_acl_ruleset_get(acl, block);
	if (IS_ERR(ruleset))
		return PTR_ERR(ruleset);

	/* increments the ruleset reference */
	rule = prestera_acl_rule_create(ruleset, f->cookie);
	if (IS_ERR(rule)) {
		err = PTR_ERR(rule);
		goto err_rule_create;
	}

	err = prestera_flower_parse(block, rule, f);
	if (err)
		goto err_rule_add;

	if (!prestera_acl_ruleset_is_offload(ruleset)) {
		err = prestera_acl_ruleset_offload(ruleset);
		if (err)
			goto err_ruleset_offload;
	}

	err = prestera_acl_rule_add(block->sw, rule);
	if (err)
		goto err_rule_add;

	prestera_acl_ruleset_put(ruleset);
	return 0;

err_ruleset_offload:
err_rule_add:
	prestera_acl_rule_destroy(rule);
err_rule_create:
	prestera_acl_ruleset_put(ruleset);
	return err;
}

void prestera_flower_destroy(struct prestera_flow_block *block,
			     struct flow_cls_offload *f)
{
	struct prestera_acl_ruleset *ruleset;
	struct prestera_acl_rule *rule;

	ruleset = prestera_acl_ruleset_lookup(block->sw->acl, block);
	if (IS_ERR(ruleset))
		return;

	rule = prestera_acl_rule_lookup(ruleset, f->cookie);
	if (rule) {
		prestera_acl_rule_del(block->sw, rule);
		prestera_acl_rule_destroy(rule);
	}
	prestera_acl_ruleset_put(ruleset);

}

int prestera_flower_stats(struct prestera_flow_block *block,
			  struct flow_cls_offload *f)
{
	struct prestera_acl_ruleset *ruleset;
	struct prestera_acl_rule *rule;
	u64 packets;
	u64 lastuse;
	u64 bytes;
	int err;

	ruleset = prestera_acl_ruleset_lookup(block->sw->acl, block);
	if (IS_ERR(ruleset))
		return PTR_ERR(ruleset);

	rule = prestera_acl_rule_lookup(ruleset, f->cookie);
	if (!rule) {
		err = -EINVAL;
		goto err_rule_get_stats;
	}

	err = prestera_acl_rule_get_stats(block->sw->acl, rule, &packets,
					  &bytes, &lastuse);
	if (err)
		goto err_rule_get_stats;

	flow_stats_update(&f->stats, bytes, packets, 0, lastuse,
			  FLOW_ACTION_HW_STATS_DELAYED);

err_rule_get_stats:
	prestera_acl_ruleset_put(ruleset);
	return err;
}
