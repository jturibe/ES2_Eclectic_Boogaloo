#include "Crypto.h"

////////////////////////////////////////////////////////////////////////////////
// BITCOIN MINER CLASS declaration in charge of performing "bitcoin mining".  //
// Finds nonces that generate a particular hash using the SHA256 algorithm.   //
////////////////////////////////////////////////////////////////////////////////

///// BITCOIN MINER CLASS
class BitcoinMiner{
private:
    uint8_t sequence[64] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)&sequence[48];
    uint64_t* nonce = (uint64_t*)&sequence[56];
    uint8_t hash[32];
    int hash_count = 0;
    SHA256 algo;

public:
    volatile uint64_t newKey;
    Mutex newKey_mutex;
    compute_hash();
};
