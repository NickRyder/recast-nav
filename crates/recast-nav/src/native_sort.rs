// Altered Rust translation of Apple's FreeBSD qsort.c and heapsort.c.
// Copyright (c) 1991, 1992, 1993 The Regents of the University of California.
// All rights reserved. Full notice: vendor/apple-libc/{qsort,heapsort}.c and
// LICENSE-BSD-3-Clause. The macOS implementation uses no libc FFI.

/// Internal CRT ordering shared by Recast contour bridges and Detour BV trees.
/// Windows/MSVC retains UCRT ordering; other non-macOS targets use glibc's
/// ordinary stable tie order.
pub fn sort_by_key<T: Clone, K: Ord + Copy>(items: &mut [T], key: impl Fn(&T) -> K) {
    #[cfg(target_os = "macos")]
    if items.len() > 1 {
        apple::quicksort(items, &key, 2 * items.len().ilog2());
    }
    #[cfg(all(target_os = "windows", target_env = "msvc"))]
    windows::sort(items, &key);
    #[cfg(not(any(target_os = "macos", all(target_os = "windows", target_env = "msvc"))))]
    items.sort_by_key(key);
}

#[cfg(all(target_os = "windows", target_env = "msvc"))]
mod windows {
    /// Reproduce the observable UCRT partition and small-array tie choices
    /// using safe record swaps. No CRT calls or byte-level access are needed.
    pub(super) fn sort<T, K: Ord + Copy>(mut items: &mut [T], key: &impl Fn(&T) -> K) {
        while items.len() > 8 {
            let end = items.len() - 1;
            let middle = items.len() / 2;
            for (a, b) in [(0, middle), (0, end), (middle, end)] {
                if key(&items[a]) > key(&items[b]) {
                    items.swap(a, b);
                }
            }
            let pivot = key(&items[middle]);
            let mut left = 0;
            let mut right = end;
            loop {
                left += 1;
                while left <= end && key(&items[left]) <= pivot {
                    left += 1;
                }
                right -= 1;
                while key(&items[right]) > pivot {
                    right -= 1;
                }
                if right < left {
                    break;
                }
                items.swap(left, right);
            }
            // Equal keys adjacent to the partition do not enter either sort.
            while right > 0 && key(&items[right]) == pivot {
                right -= 1;
            }
            let (lower, upper) = items.split_at_mut(left);
            let lower = &mut lower[..=right];
            // Recurse on the smaller partition to bound stack depth.
            if lower.len() < upper.len() {
                sort(lower, key);
                items = upper;
            } else {
                sort(upper, key);
                items = lower;
            }
        }
        // UCRT chooses the first strictly greatest item, then swaps it into
        // the last slot. In particular, two equal records exchange positions.
        for end in (1..items.len()).rev() {
            let mut greatest = 0;
            for candidate in 1..=end {
                if key(&items[candidate]) > key(&items[greatest]) {
                    greatest = candidate;
                }
            }
            items.swap(greatest, end);
        }
    }
}

#[cfg(target_os = "macos")]
mod apple {
    fn insertion<T: Clone, K: Ord + Copy>(
        items: &mut [T],
        key: &impl Fn(&T) -> K,
        limit: usize,
    ) -> bool {
        let mut swaps = 0;
        for i in 1..items.len() {
            let mut j = i;
            while j > 0 && key(&items[j - 1]) > key(&items[j]) {
                items.swap(j, j - 1);
                j -= 1;
                swaps += 1;
                if limit != 0 && swaps > limit {
                    return false;
                }
            }
        }
        true
    }

    fn median<T, K: Ord + Copy>(
        items: &[T],
        key: &impl Fn(&T) -> K,
        a: usize,
        b: usize,
        c: usize,
    ) -> usize {
        let (ka, kb, kc) = (key(&items[a]), key(&items[b]), key(&items[c]));
        if ka < kb {
            if kb < kc {
                b
            } else if ka < kc {
                c
            } else {
                a
            }
        } else if kb > kc {
            b
        } else if ka < kc {
            a
        } else {
            c
        }
    }

    pub(super) fn quicksort<T: Clone, K: Ord + Copy>(
        items: &mut [T],
        key: &impl Fn(&T) -> K,
        depth: u32,
    ) {
        let n = items.len();
        if depth == 0 {
            heapsort(items, key);
            return;
        }
        if n <= 7 {
            insertion(items, key, 0);
            return;
        }
        let (mut low, mut mid, mut high) = (0, n / 2, n - 1);
        if n > 40 {
            let step = n / 8;
            low = median(items, key, low, low + step, low + step * 2);
            mid = median(items, key, mid - step, mid, mid + step);
            high = median(items, key, high - step * 2, high - step, high);
        }
        mid = median(items, key, low, mid, high);
        items.swap(0, mid);
        let pivot = key(&items[0]);
        let (mut a, mut b, mut c, mut d) = (1, 1, n - 1, n - 1);
        let mut swapped = false;
        loop {
            while b <= c && key(&items[b]) <= pivot {
                if key(&items[b]) == pivot {
                    swapped = true;
                    items.swap(a, b);
                    a += 1;
                }
                b += 1;
            }
            while b <= c && key(&items[c]) >= pivot {
                if key(&items[c]) == pivot {
                    swapped = true;
                    items.swap(c, d);
                    d -= 1;
                }
                c -= 1;
            }
            if b > c {
                break;
            }
            items.swap(b, c);
            swapped = true;
            b += 1;
            c -= 1;
        }
        let first = a.min(b - a);
        for i in 0..first {
            items.swap(i, b - first + i);
        }
        let last = (d - c).min(n - d - 1);
        for i in 0..last {
            items.swap(b + i, n - last + i);
        }
        if !swapped && insertion(items, key, 1 + n / 4) {
            return;
        }
        let (left, right) = (b - a, d - c);
        if left > 1 {
            quicksort(&mut items[..left], key, depth - 1);
        }
        if right > 1 {
            quicksort(&mut items[n - right..], key, depth - 1);
        }
    }

    fn heapsort<T: Clone, K: Ord + Copy>(items: &mut [T], key: &impl Fn(&T) -> K) {
        let mut n = items.len();
        // Use one-based heap indices to retain the native equal-child tie policy.
        for parent in (1..=n / 2).rev() {
            let mut i = parent;
            while i * 2 <= n {
                let mut child = i * 2;
                if child < n && key(&items[child - 1]) < key(&items[child]) {
                    child += 1;
                }
                if key(&items[child - 1]) <= key(&items[i - 1]) {
                    break;
                }
                items.swap(i - 1, child - 1);
                i = child;
            }
        }
        while n > 1 {
            let displaced = items[n - 1].clone();
            items[n - 1] = items[0].clone();
            n -= 1;
            let mut i = 1;
            while i * 2 <= n {
                let mut child = i * 2;
                if child < n && key(&items[child - 1]) < key(&items[child]) {
                    child += 1;
                }
                items[i - 1] = items[child - 1].clone();
                i = child;
            }
            loop {
                let child = i;
                i = child / 2;
                if child == 1 || key(&displaced) < key(&items[i - 1]) {
                    items[child - 1] = displaced;
                    break;
                }
                items[child - 1] = items[i - 1].clone();
            }
        }
    }
}
