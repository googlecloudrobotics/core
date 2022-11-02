package oauth

import (
	"fmt"
	"math/rand"
	"sync"
	"testing"
	"time"
)

type tokenProp struct {
	token Token
	acl   string
	actAs bool
}

var (
	counter   int
	counterMu sync.Mutex
)

func randToken() *tokenProp {
	counterMu.Lock()
	defer counterMu.Unlock()
	counter += 1
	token := Token(fmt.Sprintf("%X", counter))
	return &tokenProp{token: token, acl: fmt.Sprintf("%X", counter),
		actAs: rand.Intn(2) == 1}
}

func TestTokenCacheOneTokenMultipleAcl(t *testing.T) {
	tc, err := newTokenCache(100, time.Hour)
	if err != nil {
		t.Fatal(err.Error())
	}
	tc.add("a", "acl_1", true)
	tc.add("a", "acl_2", false)
	actAs, found := tc.actAs("a", "acl_1")
	if found == false || actAs != true {
		t.Fatalf("got (%v, %v), want (true, true)", actAs, found)
	}
	actAs, found = tc.actAs("a", "acl_2")
	if found == false || actAs != false {
		t.Fatalf("got (%v, %v), want (true, false)", actAs, found)
	}
}

func TestTokenCacheExpire(t *testing.T) {
	// keys expire instantly
	tc, err := newTokenCache(100, time.Hour)
	if err != nil {
		t.Fatal(err.Error())
	}
	tc.expire *= -1
	tc.add("a", "acl_1", true)
	_, found := tc.actAs("a", "acl_1")
	if found == true {
		t.Fatalf("got key a, want not found")
	}
}

func TestEvictExpired(t *testing.T) {
	tc, err := newTokenCache(100, time.Hour)
	if err != nil {
		t.Fatal(err.Error())
	}
	tc.expire *= -1
	tc.add("a", "acl_1", true)
	tc.add("b", "acl_1", true)
	tc.add("c", "acl_1", true)
	tc.evictExpired()
	if len(tc.cache) != 0 || tc.evictOrder.Len() != 0 {
		t.Fatalf("got cache size %d and %d, want 0)",
			len(tc.cache), tc.evictOrder.Len())
	}
}

func TestEvictOldest(t *testing.T) {
	tc, err := newTokenCache(100, time.Hour)
	if err != nil {
		t.Fatal(err.Error())
	}
	tc.add("a", "acl_1", true)
	tc.add("b", "acl_1", true)
	tc.add("c", "acl_1", true)
	tc.evictOldest()
	if len(tc.cache) != 2 || tc.evictOrder.Len() != 2 {
		t.Fatalf("got cache size got %d and %d after eviction, want 2",
			len(tc.cache), tc.evictOrder.Len())
	}
	e := tc.evictOrder.Front().Value.(*entry)
	if e.key != "acl_1b" {
		t.Fatalf("got oldest element %v after eviction, want b", e)
	}
}

// Similar to test case TestEvictOldest, but without explicit eviction.
func TestEvictionGeneral(t *testing.T) {
	tc, err := newTokenCache(2, time.Hour)
	if err != nil {
		t.Fatal(err.Error())
	}
	tc.add("a", "acl_1", true)
	tc.add("b", "acl_1", true)
	tc.add("c", "acl_1", true)
	if len(tc.cache) != 2 || tc.evictOrder.Len() != 2 {
		t.Fatalf("got cache size got %d and %d after eviction, want 2",
			len(tc.cache), tc.evictOrder.Len())
	}
	e := tc.evictOrder.Front().Value.(*entry)
	if e.key != "acl_1b" {
		t.Fatalf("got oldest element %v after eviction, want b", e)
	}
}

func TestParallelAccessNoEviction(t *testing.T) {
	const workers = 3
	const entries = 10000
	tc, err := newTokenCache(workers*entries, time.Hour)
	if err != nil {
		t.Fatal(err.Error())
	}
	var wg sync.WaitGroup
	errs := make(chan error, workers)
	for i := 0; i < workers; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			err := accessWorker(tc, entries, true)
			if err != nil {
				errs <- err
			}
		}()
	}
	wg.Wait()
	if len(errs) > 0 {
		err = <-errs
		if err != nil {
			t.Fatal(err.Error())
		}
	}
}

func accessWorker(tc *tokenCache, adds int, failIfNotFound bool) error {
	// create a bunch of entries and put into cache
	tokens := make([]*tokenProp, 0, adds)
	for i := 0; i < adds; i++ {
		t := randToken()
		tokens = append(tokens, t)
		tc.add(t.token, t.acl, t.actAs)
	}
	// verify entries in the cache
	for idx, t := range tokens {
		actAs, found := tc.actAs(t.token, t.acl)
		if found == false && failIfNotFound {
			return fmt.Errorf("token %d: entry %v should be there, but is not", idx, t)
		}
		if !found {
			continue
		}
		if actAs != t.actAs {
			return fmt.Errorf("actAs got %v, want %v", actAs, t.actAs)
		}
	}
	return nil
}
