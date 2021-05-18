/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2021, Bernhard Kraemer
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/** @file
 * @brief Texas Instruments Ethernet PHY (DP83825I) driver for use with mcux stack.
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_PHY_DP83825_H_
#define ZEPHYR_DRIVERS_ETHERNET_PHY_DP83825_H_

#include "fsl_enet.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief PHY driver version */
#define FSL_PHY_DRIVER_VERSION (MAKE_VERSION(1, 0, 0)) /*!< Version 1.0.0. */

/*! @brief Defines the PHY registers. */
#define PHY_BASICCONTROL_REG 0x00U      /*!< The PHY basic control register. */
#define PHY_BASICSTATUS_REG 0x01U       /*!< The PHY basic status register. */
#define PHY_ID1_REG 0x02U               /*!< The PHY ID one register. */
#define PHY_ID2_REG 0x03U               /*!< The PHY ID two register. */
#define PHY_AUTONEG_ADVERTISE_REG 0x04U /*!< The PHY auto-negotiate advertise register. */
#define PHY_OMS_OVERRIDE_REG 0x16U      /*!< The PHY Operation Mode Strap Override register. */
#define PHY_CONTROL1_REG 0x10U          /*!< The PHY control one register. */
#define PHY_CONTROL2_REG 0x17U          /*!< The PHY control two register. */

#define PHY_CONTROL_ID1 0x22U /*!< The PHY ID1*/

/*! @brief Defines the mask flag in basic control register. */
#define PHY_BCTL_DUPLEX_MASK 0x0100U          /*!< The PHY duplex bit mask. */
#define PHY_BCTL_RESTART_AUTONEG_MASK 0x0200U /*!< The PHY restart auto negotiation mask. */
#define PHY_BCTL_AUTONEG_MASK 0x1000U         /*!< The PHY auto negotiation bit mask. */
#define PHY_BCTL_SPEED_MASK 0x2000U           /*!< The PHY speed bit mask. */
#define PHY_BCTL_LOOP_MASK 0x4000U            /*!< The PHY loop bit mask. */
#define PHY_BCTL_RESET_MASK 0x8000U           /*!< The PHY reset bit mask. */

/*!@brief Defines the mask flag of operation mode in control two register*/
#define PHY_CTL2_REMOTELOOP_MASK 0x0000U    /*!< The PHY remote loopback mask. */
#define PHY_CTL2_REFCLK_SELECT_MASK 0x0080U /*!< The PHY RMII reference clock select. */
#define PHY_CTL1_10HALFDUPLEX_MASK 0x0002U  /*!< The PHY 10M half duplex mask. */
#define PHY_CTL1_100HALFDUPLEX_MASK 0x0000U /*!< The PHY 100M half duplex mask. */
#define PHY_CTL1_10FULLDUPLEX_MASK 0x0006U  /*!< The PHY 10M full duplex mask. */
#define PHY_CTL1_100FULLDUPLEX_MASK 0x0004U /*!< The PHY 100M full duplex mask. */
#define PHY_CTL1_SPEEDUPLX_MASK 0x0006U     /*!< The PHY speed and duplex mask. */

/*! @brief Defines the mask flag in basic status register. */
#define PHY_BSTATUS_LINKSTATUS_MASK 0x0004U  /*!< The PHY link status mask. */
#define PHY_BSTATUS_AUTONEGABLE_MASK 0x0008U /*!< The PHY auto-negotiation ability mask. */
#define PHY_BSTATUS_AUTONEGCOMP_MASK 0x0020U /*!< The PHY auto-negotiation complete mask. */

/*! @brief Defines the mask flag in PHY auto-negotiation advertise register. */
#define PHY_100BaseT4_ABILITY_MASK 0x200U    /*!< The PHY have the T4 ability. */
#define PHY_100BASETX_FULLDUPLEX_MASK 0x100U /*!< The PHY has the 100M full duplex ability.*/
#define PHY_100BASETX_HALFDUPLEX_MASK 0x080U /*!< The PHY has the 100M full duplex ability.*/
#define PHY_10BASETX_FULLDUPLEX_MASK 0x040U  /*!< The PHY has the 10M full duplex ability.*/
#define PHY_10BASETX_HALFDUPLEX_MASK 0x020U  /*!< The PHY has the 10M full duplex ability.*/

/*! @brief Defines the mask flag in PHY Operation Mode Strap Override/Status register. */
#define PHY_OMS_NANDTREE_MASK 0x0U     /*!< The PHY NAND Tree Strap-In Override/Status mask. */
#define PHY_OMS_FACTORY_MODE_MASK 0x0U /*!< The factory mode Override/Status mask. */

/*! @brief Defines the PHY status. */
enum _phy_status {
	kStatus_PHY_SMIVisitTimeout = MAKE_STATUS(kStatusGroup_PHY, 1),
	kStatus_PHY_AutoNegotiateFail = MAKE_STATUS(kStatusGroup_PHY, 2)
};

/*! @brief Defines the PHY link speed. This is align with the speed for ENET MAC. */
#define phy_speed_t uint8_t
enum {
	kPHY_Speed10M = 0U,
	kPHY_Speed100M
};

/*! @brief Defines the PHY link duplex. */
#define phy_duplex_t uint8_t
enum {
	kPHY_HalfDuplex = 0U,
	kPHY_FullDuplex
};

/*! @brief Defines the PHY loopback mode. */
#define phy_loop_t uint8_t
enum {
	kPHY_LocalLoop = 0U,
	kPHY_RemoteLoop
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes PHY. This function initialize the SMI interface and initialize PHY.
 * The SMI is the MII management interface between PHY and MAC, which should be
 * firstly initialized before any other operation for PHY.
 *
 * @param base        ENET peripheral base address.
 * @param phyAddr     The PHY address.
 * @param srcClock_Hz The module clock frequency - system clock for MII management interface - SMI.
 * @retval kStatus_Success  PHY initialize success
 * @retval kStatus_PHY_SMIVisitTimeout  PHY SMI visit time out
 * @retval kStatus_PHY_AutoNegotiateFail  PHY auto negotiate fail
 */
status_t PHY_Init(ENET_Type * base, uint32_t phyAddr, uint32_t srcClock_Hz);

/*!
 * @brief PHY Write function. This function write data over the SMI to
 * the specified PHY register. This function is called by all PHY interfaces.
 *
 * @param base    ENET peripheral base address.
 * @param phyAddr The PHY address.
 * @param phyReg  The PHY register.
 * @param data    The data written to the PHY register.
 * @retval kStatus_Success     PHY write success
 * @retval kStatus_PHY_SMIVisitTimeout  PHY SMI visit time out
 */
status_t PHY_Write(ENET_Type *base, uint32_t phyAddr, uint32_t phyReg, uint32_t data);

/*!
 * @brief PHY Read function. This interface read data over the SMI from the
 * specified PHY register. This function is called by all PHY interfaces.
 *
 * @param base     ENET peripheral base address.
 * @param phyAddr  The PHY address.
 * @param phyReg   The PHY register.
 * @param dataPtr  The address to store the data read from the PHY register.
 * @retval kStatus_Success  PHY read success
 * @retval kStatus_PHY_SMIVisitTimeout  PHY SMI visit time out
 */
status_t PHY_Read(ENET_Type *base, uint32_t phyAddr, uint32_t phyReg, uint32_t *dataPtr);

/*!
 * @brief Enables/disables PHY loopback.
 *
 * @param base     ENET peripheral base address.
 * @param phyAddr  The PHY address.
 * @param mode     The loopback mode to be enabled, please see "phy_loop_t".
 * the two loopback mode should not be both set. when one loopback mode is set
 * the other one should be disabled.
 * @param enable   True to enable, false to disable.
 * @retval kStatus_Success  PHY loopback success
 * @retval kStatus_PHY_SMIVisitTimeout  PHY SMI visit time out
 */
status_t PHY_EnableLoopback(ENET_Type *base, uint32_t phyAddr, phy_loop_t mode, bool enable);

/*!
 * @brief Gets the PHY link status.
 *
 * @param base     ENET peripheral base address.
 * @param phyAddr  The PHY address.
 * @param status   The link up or down status of the PHY.
 *         - true the link is up.
 *         - false the link is down.
 * @retval kStatus_Success   PHY get link status success
 * @retval kStatus_PHY_SMIVisitTimeout  PHY SMI visit time out
 */
status_t PHY_GetLinkStatus(ENET_Type *base, uint32_t phyAddr, bool *status);

/*!
 * @brief Gets the PHY link speed and duplex.
 *
 * @param base     ENET peripheral base address.
 * @param phyAddr  The PHY address.
 * @param speed    The address of PHY link speed.
 * @param duplex   The link duplex of PHY.
 * @retval kStatus_Success   PHY get link speed and duplex success
 * @retval kStatus_PHY_SMIVisitTimeout  PHY SMI visit time out
 */
status_t PHY_GetLinkSpeedDuplex(ENET_Type *base,
								uint32_t phyAddr,
								phy_speed_t *speed,
								phy_duplex_t *duplex);

#if defined(__cplusplus)
}
#endif

#endif /* ZEPHYR_DRIVERS_ETHERNET_PHY_DP83825_H_ */
