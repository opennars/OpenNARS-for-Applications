/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "HashTable.h"

void *HashTable_Get(HashTable *hashtable, void *key)
{
    HASH_TYPE keyhash = hashtable->hash(key) % hashtable->buckets;
    VMItem *item = hashtable->HT[keyhash];
    for(; item!=NULL; item=item->next)
    {
        if(hashtable->equal(item->key, key))
        {
            return item->value;
        }
    }
    return NULL;
}

void HashTable_Set(HashTable *hashtable, void *key, void *value)
{
    HASH_TYPE keyhash = hashtable->hash(key) % hashtable->buckets;
    //Check if item already exists in hashtable, if yes return
    VMItem *item = hashtable->HT[keyhash];
    bool empty = item == NULL;
    if(!empty)
    {
        for(; item!=NULL;)
        {
            if(hashtable->equal(item->key, key))
            {
                item->value = value;
                return;
            }
            if(item->next == NULL)
                break;
            else
                item=item->next;
        }
    }
    //Retrieve recycled VMItem from the stack and set its value to c
    VMItem *popped = Stack_Pop(&hashtable->VMStack);
    popped->value = value;
    popped->key = key;
    popped->next = NULL;
    //Case1: HT at hash was empty so add recycled item at HT[keyhash]
    if(empty)
    {
        hashtable->HT[keyhash] = popped;
    }
    //Case2: HT at hash not empty so add recycled item at end of the chain of HT[keyhash]
    else
    {
        assert(item != NULL, "VMItem should not be null!");
        item->next = popped;
    }
}

void HashTable_Delete(HashTable *hashtable, void *key)
{
    HASH_TYPE keyhash = hashtable->hash(key) % hashtable->buckets;
    VMItem *item = hashtable->HT[keyhash];
    VMItem *previous = NULL;
    //If there is only one item set HT[keyhash] to NULL and push back the VMItem to stack for recycling
    if(item->next == NULL)
    {
        hashtable->HT[keyhash] = NULL;
        Stack_Push(&hashtable->VMStack, item);
        return;
    }
    //If there is more than 1 item, we have to remove the item from chain, relinking previous to next
    for(; item!=NULL; previous=item, item=item->next)
    {
        //item found?
        if(hashtable->equal(item->key, key))
        {
            //remove item and return
            if(previous == NULL)
            {
                hashtable->HT[keyhash] = item->next;
            }
            else
            {
                previous->next = item->next;
            }
            Stack_Push(&hashtable->VMStack, item);
            return;
        }
    }
    assert(false, "HashTable deletion failed, item was not found!");
}

void HashTable_INIT(HashTable *hashtable, VMItem* storage, VMItem** storageptrs, VMItem** HT, int buckets, int maxElements, Equal equal, Hash hash)
{
    hashtable->storage = storage;
    hashtable->storageptrs = storageptrs;
    hashtable->HT = HT;
    hashtable->VMStack = (Stack) {0};
    Stack_INIT(&hashtable->VMStack, (void**) hashtable->storageptrs, maxElements);
    hashtable->equal = equal;
    hashtable->hash = hash;
    hashtable->buckets = buckets;
    for(int i=0; i<buckets; i++)
    {
        hashtable->HT[i] = NULL;
    }
    for(int i=0; i<maxElements; i++)
    {
        hashtable->storage[i] = (VMItem) {0};
        hashtable->storageptrs[i] = NULL;
        Stack_Push(&hashtable->VMStack, &hashtable->storage[i]);
    }
}

int HashTable_MaximumChainLength(HashTable *hashtable)
{
    int maxlen = 0;
    for(int i=0; i<hashtable->buckets; i++)
    {
        VMItem *item = hashtable->HT[i];
        int cnt = 0;
        for(;item != NULL; item=item->next, cnt++);
        maxlen = MAX(maxlen, cnt);
    }
    return maxlen;
}
