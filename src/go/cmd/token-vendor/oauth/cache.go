package oauth

import (
	"container/list"
	"fmt"
	"sync"
	"time"
)

type entry struct {
	// when the cache entry expires
	expires time.Time
	// the cache key
	key string
	// if the key has actAs permission
	actAs bool
}

type tokenCache struct {
	// hashmap for quick key lookup
	cache map[string]*list.Element
	// list of entries sorted by expire time, oldest in front
	evictOrder *list.List
	// mutex to serialize cache access
	mu sync.Mutex
	// expire an entry after
	expire time.Duration
	// maximum number of cache entries to keep
	size int
}

// Creates a simple cache for the verifier logic with a maximum size.
//
// Eviction is done lazily during add. Existing entries are refreshed on add,
// but not on access.
func newTokenCache(size int, expire time.Duration) (*tokenCache, error) {
	if size < 1 {
		return nil, fmt.Errorf("invalid cache size %d", size)
	}
	if expire <= 0 {
		return nil, fmt.Errorf("expire must be positive, got %v", expire)
	}
	return &tokenCache{cache: make(map[string]*list.Element, size),
		size: size, expire: expire, evictOrder: list.New()}, nil
}

// add if a given token has `actAs` permission on a given service account.
//
// If the entry exists already, refresh it. Thread-safe.
func (c *tokenCache) add(token Token, sa string, actAs bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	// evict all expired entries
	c.evictExpired()
	key := sa + string(token)
	expires := time.Now().Add(c.expire)
	// for existing keys update the entry
	if el, found := c.cache[key]; found {
		e := el.Value.(*entry)
		e.actAs = actAs
		e.expires = expires
		c.evictOrder.MoveToBack(el)
		return
	}
	// if we reached capacity, evict oldest element
	if len(c.cache) == c.size {
		c.evictOldest()
	}
	// create a new cache entry
	e := &entry{expires: expires, key: key, actAs: actAs}
	el := c.evictOrder.PushBack(e)
	c.cache[key] = el
}

// Evict all expired keys (not thread-safe)
func (c *tokenCache) evictExpired() {
	for el := c.evictOrder.Front(); el != nil; el = c.evictOrder.Front() {
		e := el.Value.(*entry)
		if e.expires.After(time.Now()) {
			return
		}
		c.evictOrder.Remove(el)
		delete(c.cache, e.key)
	}
}

// Evict oldest cache entry (not thread-safe)
func (c *tokenCache) evictOldest() {
	front := c.evictOrder.Front()
	if front == nil {
		return
	}
	e := front.Value.(*entry)
	c.evictOrder.Remove(front)
	delete(c.cache, e.key)
}

// actAs queries the cache if a token has actAs permission on a service account.
//
// Returns (actAs, found). If found is false, token+sa was not found in cache
// or the entry was already expired. Thread-safe.
func (c *tokenCache) actAs(token Token, sa string) (bool, bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	key := sa + string(token)
	if el, found := c.cache[key]; found {
		e := el.Value.(*entry)
		// entry is already expired
		if e.expires.Before(time.Now()) {
			return false, false
		}
		return e.actAs, true
	}
	return false, false
}
