/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018-2019, NVIDIA CORPORATION. All rights reserved. */

#ifndef DT_BINDINGS_CLOCK_TEGRA234_CLOCK_H
#define DT_BINDINGS_CLOCK_TEGRA234_CLOCK_H

/**
 * @file
 * @defgroup bpmp_clock_ids Clock ID's
 * @{
 */
/** @brief output of gate CLK_ENB_FUSE */
#define TEGRA234_CLK_FUSE			40U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4 */
#define TEGRA234_CLK_SDMMC4			123U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTA */
#define TEGRA234_CLK_UARTA			155U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC_LEGACY_TM switch divider output */
#define TEGRA234_CLK_SDMMC_LEGACY_TM		219U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLC4_BASE */
#define TEGRA234_CLK_PLLC4			237U
/** @brief 32K input clock provided by PMIC */
#define TEGRA234_CLK_CLK_32K			289U

#endif
