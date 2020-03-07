#include "mining_bitcoin.h"

void BitcoinMiner::compute_hash(){
    newKey_mutex.lock();
    *key = newKey;
    newKey_mutex.unlock();
    algo.computeHash(hash, sequence, 64);
    if ((hash[0]==0) && (hash[1]==0)) {
        char message[100];
        sprintf(message, "Successful nonce, Hex rep: 0x%X\n\r", *nonce);
        putMessage(message);
    }
    (*nonce)++;
}
