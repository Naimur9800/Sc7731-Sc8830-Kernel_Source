/**************************************************************
* Copyright 2014 (c) Discretix Technologies Ltd.              *
* This software is protected by copyright, international      *
* treaties and various patents. Any copy or reproduction of   *
* the software as permitted below, must include this          *
* Copyright Notice as well as any other notices provided      *
* under the relevant license.                                 *
*                                                             *
* This software shall be governed by, and may be used and     *
* redistributed under the terms and conditions of the GNU     *
* General Public License version 2, as published by the       *
* Free Software Foundation.                                   *
* This software is distributed in the hope that it will be    *
* useful, but WITHOUT ANY liability and WARRANTY; without     *
* even the implied warranty of MERCHANTABILITY or FITNESS     *
* FOR A PARTICULAR PURPOSE. See the GNU General Public        *
* License for more details.                                   *
* You should have received a copy of the GNU General          *
* Public License along with this software; if not, please     *
* write to the Free Software Foundation, Inc.,                *
* 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.   *
**************************************************************/


#ifndef __SECURE_KEY_DEFS_H__
#define __SECURE_KEY_DEFS_H__


/******************************************************************************
*				DEFINITIONS
******************************************************************************/

#define DX_SECURE_KEY_PACKAGE_BUF_SIZE_IN_BYTES 	112

#define DX_SECURE_KEY_NONCE_SIZE_IN_BYTES		12

#define DX_SECURE_KEY_MULTI2_MIN_ROUNDS			8
#define DX_SECURE_KEY_MULTI2_MAX_ROUNDS			128
#define DX_SECURE_KEY_MAX_CTR_RANGE_VALUE		0x10000

/******************************************************************************
*				TYPE DEFINITIONS
******************************************************************************/
enum secure_key_type {
	DX_SECURE_KEY_AES_KEY128  = 0,
	DX_SECURE_KEY_AES_KEY256  = 1,
	DX_SECURE_KEY_MULTI2 	  = 2,
	DX_SECURE_KEY_BYPASS 	  = 3,
	DX_SECURE_KEY_MAINTENANCE  = 7,
	DX_SECURE_KEY_KSIZE_RESERVE32B = INT32_MAX
};

enum secure_key_cipher_mode {
	DX_SECURE_KEY_CIPHER_CBC = 1,
	DX_SECURE_KEY_CIPHER_CTR = 2,
	DX_SECURE_KEY_CIPHER_OFB = 6,
	DX_SECURE_KEY_CIPHER_CTR_NONCE_CTR_PROT_NSP = 9,
	DX_SECURE_KEY_CIPHER_CTR_NONCE_PROT = 10,
	DX_SECURE_KEY_CIPHER_CBC_CTS = 11,
	DX_SECURE_KEY_CIPHER_RESERVE32B = INT32_MAX
};

enum secure_key_direction {
	DX_SECURE_KEY_DIRECTION_ENCRYPT = 0,
	DX_SECURE_KEY_DIRECTION_DECRYPT = 1,
	DX_SECURE_KEY_DIRECTION_RESERVE32B = INT32_MAX
};


typedef uint8_t skeyPackageBuf_t[DX_SECURE_KEY_PACKAGE_BUF_SIZE_IN_BYTES];

typedef uint8_t skeyNonceBuf_t[DX_SECURE_KEY_NONCE_SIZE_IN_BYTES];


#endif /*__SECURE_KEY_DEFS_H__*/


