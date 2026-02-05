
# Iluvatar Style

## The Essence Of Style

> "There are three things extremely hard: steel, a diamond, and to know one's self." — Benjamin
> Franklin

Iluvatar's coding style is evolving. A collective give-and-take at the intersection of engineering
and art. Numbers and human intuition. Reason and experience. First principles and knowledge.
Precision and poetry. Just like music. A tight beat. A rare groove. Words that rhyme and rhymes that
break. Biodigital jazz. This is what we've learned along the way. The best is yet to come.

## Why Have Style?

Another word for style is design.

> "The design is not just what it looks like and feels like. The design is how it works." — Steve
> Jobs

Our design goals are safety, performance, and developer experience. In that order. All three are
important. Good style advances these goals. Does the code make for more or less safety, performance
or developer experience? That is why we need style.

Put this way, style is more than readability, and readability is table stakes, a means to an end
rather than an end in itself.

> "...in programming, style is not something to pursue directly. Style is necessary only where
> understanding is missing." ─ [Let Over
> Lambda](https://letoverlambda.com/index.cl/guest/chap1.html)

This document explores how we apply these design goals to coding style. First, a word on simplicity,
elegance and technical debt.

## On Simplicity And Elegance

Simplicity is not a free pass. It's not in conflict with our design goals. It need not be a
concession or a compromise.

Rather, simplicity is how we bring our design goals together, how we identify the "super idea" that
solves the axes simultaneously, to achieve something elegant.

> "Simplicity and elegance are unpopular because they require hard work and discipline to achieve" —
> Edsger Dijkstra

Contrary to popular belief, simplicity is also not the first attempt but the hardest revision. It's
easy to say "let's do something simple", but to do that in practice takes thought, multiple passes,
many sketches, and still we may have to ["throw one
away"](https://en.wikipedia.org/wiki/The_Mythical_Man-Month).

The hardest part, then, is how much thought goes into everything.

We spend this mental energy upfront, proactively rather than reactively, because we know that when
the thinking is done, what is spent on the design will be dwarfed by the implementation and testing,
and then again by the costs of operation and maintenance.

An hour or day of design is worth weeks or months in production:

> "the simple and elegant systems tend to be easier and faster to design and get right, more
> efficient in execution, and much more reliable" — Edsger Dijkstra

## Technical Debt

What could go wrong? What's wrong? Which question would we rather ask? The former, because code,
like steel, is less expensive to change while it's hot. A problem solved in production is many times
more expensive than a problem solved in implementation, or a problem solved in design.

Since it's hard enough to discover showstoppers, when we do find them, we solve them. We don't allow
potential memcpy latency spikes, or exponential complexity algorithms to slip through.

> "You shall not pass!" — Gandalf

In other words, Iluvatar has a "zero technical debt" policy. We do it right the first time. This is
important because the second time may not transpire, and because doing good work, that we can be
proud of, builds momentum.

We know that what we ship is solid. We may lack crucial features, but what we have meets our design
goals. This is the only way to make steady incremental progress, knowing that the progress we have
made is indeed progress.

## Safety

> "The rules act like the seat-belt in your car: initially they are perhaps a little uncomfortable,
> but after a while their use becomes second-nature and not using them becomes unimaginable." —
> Gerard J. Holzmann

[NASA's Power of Ten — Rules for Developing Safety Critical
Code](https://spinroot.com/gerard/pdf/P10.pdf) will change the way you code forever. To expand:

- Use **only very simple, explicit control flow** for clarity. **Do not use recursion** to ensure
  that all executions that should be bounded are bounded. Use **only a minimum of excellent
  abstractions** but only if they make the best sense of the domain. Abstractions are [never zero
  cost](https://isaacfreund.com/blog/2022-05/). Every abstraction introduces the risk of a leaky
  abstraction.

- **Put a limit on everything** because, in reality, this is what we expect—everything has a limit.
  For example, all loops and all queues must have a fixed upper bound to prevent infinite loops or
  tail latency spikes. This follows the ["fail-fast"](https://en.wikipedia.org/wiki/Fail-fast)
  principle so that violations are detected sooner rather than later. Where a loop cannot terminate
  (e.g. an event loop), this must be asserted.

- Use explicitly-sized integer types (`u32`, `u64`, `i32`, etc.) for domain data where the size is
  semantically meaningful. Reserve `usize` for indexing and lengths where Rust idiom requires it.
  Avoid using `usize` to represent domain quantities like counts, identifiers, or measurements.

- **Assertions detect programmer errors. Unlike operating errors, which are expected and which must
  be handled, assertion failures are unexpected. The only correct way to handle corrupt code is to
  crash. Assertions downgrade catastrophic correctness bugs into liveness bugs. Assertions are a
  force multiplier for discovering bugs by fuzzing.**

  - **Assert all function arguments and return values, pre/postconditions and invariants.** A
    function must not operate blindly on data it has not checked. The purpose of a function is to
    increase the probability that a program is correct. Assertions within a function are part of how
    functions serve this purpose. The assertion density of the code must average a minimum of two
    assertions per function.

  - **[Pair assertions](https://tigerbeetle.com/blog/2023-12-27-it-takes-two-to-contract).** For
    every property you want to enforce, try to find at least two different code paths where an
    assertion can be added. For example, assert validity of data right before writing it to disk,
    and also immediately after reading from disk.

  - On occasion, you may use a blatantly true assertion instead of a comment as stronger
    documentation where the assertion condition is critical and surprising.

  - Split compound assertions: prefer `assert!(a); assert!(b);` over `assert!(a && b);`. The
    former is simpler to read, and provides more precise information if the condition fails.

  - Use `debug_assert!` for assertions that are expensive to evaluate but valuable during
    development. Use `assert!` for cheap checks that should remain in release builds. When in
    doubt, prefer `assert!`.

  - **Assert the relationships of compile-time constants** using `const` assertions or
    `static_assert!`-style checks. Compile-time assertions are extremely powerful because they are
    able to check a program's design integrity _before_ the program even executes.

  - **The golden rule of assertions is to assert the _positive space_ that you do expect AND to
    assert the _negative space_ that you do not expect** because where data moves across the
    valid/invalid boundary between these spaces is where interesting bugs are often found. This is
    also why **tests must test exhaustively**, not only with valid data but also with invalid data,
    and as valid data becomes invalid.

  - Assertions are a safety net, not a substitute for human understanding. With testing, there is
    the temptation to trust the fuzzer. But a fuzzer can prove only the presence of bugs, not their
    absence. Therefore:
    - Build a precise mental model of the code first,
    - encode your understanding in the form of assertions,
    - write the code and comments to explain and justify the mental model to your reviewer,
    - and use fuzzing as the final line of defense, to find bugs in your and reviewer's
      understanding of code.

- All memory should be allocated at startup where possible. **Minimize dynamic allocation after
  initialization.** Pre-allocate buffers, use `Vec::with_capacity`, and prefer fixed-size arrays
  over growable collections. This avoids unpredictable behavior that can significantly affect
  performance. As a second-order effect, it is our experience that this also makes for more
  efficient, simpler designs that are more performant and easier to maintain and reason about,
  compared to designs that do not consider all possible memory usage patterns upfront as part of the
  design.

- Declare variables at the **smallest possible scope**, and **minimize the number of variables in
  scope**, to reduce the probability that variables are misused.

- There's a sharp discontinuity between a function fitting on a screen, and having to scroll to
  see how long it is. For this physical reason we enforce a **hard limit of 70 lines per function**.
  Art is born of constraints. There are many ways to cut a wall of code into chunks of 70 lines,
  but only a few splits will feel right. Some rules of thumb:

  * Good function shape is often the inverse of an hourglass: a few parameters, a simple return
    type, and a lot of meaty logic between the braces.
  * Centralize control flow. When splitting a large function, try to keep all match/if statements
    in the "parent" function, and move non-branchy logic fragments to helper functions. Divide
    responsibility. All control flow should be handled by _one_ function, the rest shouldn't care
    about control flow at all. In other words,
    ["push `if`s up and `for`s down"](https://matklad.github.io/2023/11/15/push-ifs-up-and-fors-down.html).
  * Similarly, centralize state manipulation. Let the parent function keep all relevant state in
    local variables, and use helpers to compute what needs to change, rather than applying the
    change directly. Keep leaf functions pure.

- Appreciate, from day one, **all compiler warnings at the compiler's strictest setting**. Run
  `cargo clippy` and treat its warnings as errors.

- Whenever your program has to interact with external entities, **don't do things directly in
  reaction to external events**. Instead, your program should run at its own pace. Not only does
  this make your program safer by keeping the control flow of your program under your control, it
  also improves performance for the same reason (you get to batch, instead of context switching on
  every event). Additionally, this makes it easier to maintain bounds on work done per time period.

Beyond these rules:

- Compound conditions that evaluate multiple booleans make it difficult for the reader to verify
  that all cases are handled. Split compound conditions into simple conditions using nested
  `if/else` branches. Split complex `else if` chains into nested `else { if { } }` trees. This
  makes the branches and cases clear. Again, consider whether a single `if` does not also need a
  matching `else` branch, to ensure that the positive and negative spaces are handled or asserted.

- Negations are not easy! State invariants positively. When working with lengths and indexes, this
  form is easy to get right (and understand):

  ```rust
  if index < length {
      // The invariant holds.
  } else {
      // The invariant doesn't hold.
  }
  ```

  This form is harder, and also goes against the grain of how `index` would typically be compared to
  `length`, for example, in a loop condition:

  ```rust
  if index >= length {
      // It's not true that the invariant holds.
  }
  ```

- All errors must be handled. Do not use `.unwrap()` or `.expect()` in production code paths unless
  the invariant has been asserted or is provably safe. Prefer `?` propagation or explicit `match`.
  An [analysis of production failures in distributed data-intensive
  systems](https://www.usenix.org/system/files/conference/osdi14/osdi14-paper-yuan.pdf) found that
  the majority of catastrophic failures could have been prevented by simple testing of error
  handling code.

> "Specifically, we found that almost all (92%) of the catastrophic system failures are the result
> of incorrect handling of non-fatal errors explicitly signaled in software."

- **Always motivate, always say why**. Never forget to say why. Because if you explain the rationale
  for a decision, it not only increases the hearer's understanding, and makes them more likely to
  adhere or comply, but it also shares criteria with them with which to evaluate the decision and
  its importance.

- **Explicitly pass options to library functions at the call site, instead of relying on the
  defaults**. For example, use builder patterns with all fields specified, or pass explicit config
  structs rather than relying on `Default::default()`. This improves readability but most of all
  avoids latent, potentially catastrophic bugs in case the library ever changes its defaults.

## Performance

> "The lack of back-of-the-envelope performance sketches is the root of all evil." — Rivacindela
> Hudsoni

- Think about performance from the outset, from the beginning. **The best time to solve performance,
  to get the huge 1000x wins, is in the design phase, which is precisely when we can't measure or
  profile.** It's also typically harder to fix a system after implementation and profiling, and the
  gains are less. So you have to have mechanical sympathy. Like a carpenter, work with the grain.

- **Perform back-of-the-envelope sketches with respect to the four resources (network, disk, memory,
  CPU) and their two main characteristics (bandwidth, latency).** Sketches are cheap. Use sketches
  to be "roughly right" and land within 90% of the global maximum.

- Optimize for the slowest resources first (network, disk, memory, CPU) in that order, after
  compensating for the frequency of usage, because faster resources may be used many times more. For
  example, a memory cache miss may be as expensive as a disk fsync, if it happens many times more.

- Distinguish between the control plane and data plane. A clear delineation between control plane
  and data plane through the use of batching enables a high level of assertion safety without losing
  performance.

- Amortize network, disk, memory and CPU costs by batching accesses.

- Let the CPU be a sprinter doing the 100m. Be predictable. Don't force the CPU to zig zag and
  change lanes. Give the CPU large enough chunks of work. This comes back to batching.

- Be explicit. Minimize dependence on the compiler to do the right thing for you.

  In particular, extract hot loops into stand-alone functions with primitive arguments without
  `&self`. That way, the compiler doesn't need to prove that it can cache struct fields in
  registers, and a human reader can spot redundant computations easier.

## Developer Experience

> "There are only two hard things in Computer Science: cache invalidation, naming things, and
> off-by-one errors." — Phil Karlton

### Naming Things

- **Get the nouns and verbs just right.** Great names are the essence of great code, they capture
  what a thing is or does, and provide a crisp, intuitive mental model. They show that you
  understand the domain. Take time to find the perfect name, to find nouns and verbs that work
  together, so that the whole is greater than the sum of its parts.

- Follow Rust naming conventions: `snake_case` for functions, variables, modules, and file names;
  `CamelCase` for types and traits. The underscore in `snake_case` is the closest thing we have as
  programmers to a space, and helps to separate words and encourage descriptive names.

- Do not abbreviate variable names, unless the variable is a primitive integer type used as an
  argument to a sort function or matrix calculation. Use long form arguments in scripts: `--force`,
  not `-f`. Single letter flags are for interactive usage.

- For the rest, follow the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/).

- Add units or qualifiers to variable names, and put the units or qualifiers last, sorted by
  descending significance, so that the variable starts with the most significant word, and ends with
  the least significant word. For example, `latency_ms_max` rather than `max_latency_ms`. This will
  then line up nicely when `latency_ms_min` is added, as well as group all variables that relate to
  latency.

- Infuse names with meaning. For example, `buffer: Vec<u8>` is a good, if boring name, but
  `frame_buffer: Vec<u8>` and `send_buffer: Vec<u8>` are excellent. They inform the reader about
  purpose and lifecycle.

- When choosing related names, try hard to find names with the same number of characters so that
  related variables all line up in the source. For example, as arguments to a copy function,
  `source` and `target` are better than `src` and `dest` because they have the second-order effect
  that any related variables such as `source_offset` and `target_offset` will all line up in
  calculations and slices. This makes the code symmetrical, with clean blocks that are easier for
  the eye to parse and for the reader to check.

- When a single function calls out to a helper function or callback, prefix the name of the helper
  function with the name of the calling function to show the call history. For example,
  `read_sector()` and `read_sector_callback()`.

- Callbacks go last in the list of parameters. This mirrors control flow: callbacks are also
  _invoked_ last. In Rust, closures and `impl Fn` arguments should come last.

- _Order_ matters for readability (even if it doesn't affect semantics). On the first read, a file
  is read top-down, so put important things near the top. The `main` function goes first.

  The same goes for structs, the order is fields then trait implementations then methods:

  ```rust
  struct Tracer {
      time: Time,
      process_id: ProcessId,
  }

  impl Tracer {
      fn new(time: Time) -> Self {
          // ...
      }
  }
  ```

  If a nested type is complex, make it a top-level type in its own module.

  At the same time, not everything has a single right order. When in doubt, consider sorting
  alphabetically, taking advantage of big-endian naming.

- Don't overload names with multiple meanings that are context-dependent. For example, a "frame"
  could mean a video frame or a data frame. Pick distinct, unambiguous names, or qualify them:
  `video_frame` and `wire_frame`.

- Think of how names will be used outside the code, in documentation or communication. For example,
  a noun is often a better descriptor than an adjective or present participle, because a noun can be
  directly used in correspondence without having to be rephrased. Compare `camera.pipeline` vs
  `camera.processing`. The former can be used directly as a section header in a document or
  conversation, whereas the latter must be clarified. Noun names compose more clearly for derived
  identifiers, e.g. `config.pipeline_max`.

- When a function takes multiple arguments of the same type that could be mixed up, use a config
  struct or newtype wrappers. A function taking two `u64` parameters must use a struct or newtypes
  to distinguish them. If an argument can be `None`, it should be named so that the meaning of
  `None` at the call site is clear.

- **Write descriptive commit messages** that inform and delight the reader, because your commit
  messages are being read. Note that a pull request description is not stored in the git repository
  and is invisible in `git blame`, and therefore is not a replacement for a commit message.

- Don't forget to say why. Code alone is not documentation. Use comments to explain why you wrote
  the code the way you did. Show your workings.

- Don't forget to say how. For example, when writing a test, think of writing a description at the
  top to explain the goal and methodology of the test, to help your reader get up to speed, or to
  skip over sections, without forcing them to dive in.

- Comments are sentences, with a space after the slash, with a capital letter and a full stop, or a
  colon if they relate to something that follows. Comments are well-written prose describing the
  code, not just scribblings in the margin. Comments after the end of a line _can_ be phrases, with
  no punctuation.

### Cache Invalidation

- Don't duplicate variables or take aliases to them. This will reduce the probability that state
  gets out of sync. Rust's ownership model helps here, but be vigilant with `Clone` and interior
  mutability (`RefCell`, `Arc<Mutex<_>>`) which reintroduce this class of bug.

- For large structs, pass by `&T` reference rather than by value. This avoids accidental copies on
  the stack. Derive `Clone` intentionally, not by default — if a type shouldn't be casually copied,
  don't make it easy.

- **Shrink the scope** to minimize the number of variables at play and reduce the probability that
  the wrong variable is used.

- Calculate or check variables close to where/when they are used. **Don't introduce variables before
  they are needed.** Don't leave them around where they are not. This will reduce the probability of
  a POCPOU (place-of-check to place-of-use), a distant cousin to the infamous
  [TOCTOU](https://en.wikipedia.org/wiki/Time-of-check_to_time-of-use). Most bugs come down to a
  semantic gap, caused by a gap in time or space, because it's harder to check code that's not
  contained along those dimensions.

- Use simpler function signatures and return types to reduce dimensionality at the call site, the
  number of branches that need to be handled at the call site, because this dimensionality can also
  be viral, propagating through the call chain. For example, as a return type, `()` trumps `bool`,
  `bool` trumps `u64`, `u64` trumps `Option<u64>`, and `Option<u64>` trumps `Result<u64>`.

- Ensure that functions run to completion without suspending, so that precondition assertions are
  true throughout the lifetime of the function. Be cautious with `.await` points — assertions
  established before an `.await` may no longer hold after it resumes.

- Be on your guard for **[buffer bleeds](https://en.wikipedia.org/wiki/Heartbleed)**. This is a
  buffer underflow, the opposite of a buffer overflow, where a buffer is not fully utilized, with
  padding not zeroed correctly. This may not only leak sensitive information, but may cause
  deterministic guarantees to be violated.

- Use newlines to **group resource allocation and cleanup**, i.e. before the resource allocation
  and after the corresponding `Drop` guard or scope, to make leaks easier to spot.

### Off-By-One Errors

- **The usual suspects for off-by-one errors are casual interactions between an `index`, a `count`
  or a `size`.** These are all primitive integer types, but should be seen as distinct types, with
  clear rules to cast between them. To go from an `index` to a `count` you need to add one, since
  indexes are _0-based_ but counts are _1-based_. To go from a `count` to a `size` you need to
  multiply by the unit. Again, this is why including units and qualifiers in variable names is
  important.

- Show your intent with respect to division. For example, use `checked_div()`, `wrapping_div()`,
  or `div_ceil()` to show the reader you've thought through all the interesting scenarios where
  rounding or overflow may be involved. Prefer explicit arithmetic methods over bare `/` when the
  edge cases matter.

### Style By The Numbers

- Run `cargo fmt`.

- Use 4 spaces of indentation, rather than 2 spaces, as that is more obvious to the eye at a
  distance.

- Hard limit all line lengths, without exception, to at most 100 columns for a good typographic
  "measure". Use it up. Never go beyond. Nothing should be hidden by a horizontal scrollbar. Let
  your editor help you by setting a column ruler. Use `rustfmt` configuration to enforce this.

  Similar to function length, the motivation behind the number 100 is physical: just enough
  to fit two copies of the code side-by-side on a screen.

- Add braces to the `if` statement always. Rust enforces this syntactically, which is one less
  class of "goto fail;" bugs to worry about.

### Dependencies

Iluvatar has a **minimal dependencies policy**. Every dependency is a liability — a supply chain
attack surface, a safety and performance risk, and a build time cost. Before adding a dependency,
ask: can we write this ourselves in a reasonable amount of code? If the answer is yes, do that. The
dependencies we do take must earn their place by being well-maintained, widely trusted, and genuinely
hard to replicate (e.g. `smol`, `quinn`, `v4l`).

### Tooling

Similarly, tools have costs. A small standardized toolbox is simpler to operate than an array of
specialized instruments each with a dedicated manual. Our primary tool is Rust and `cargo`. It may
not be the best for everything, but it's good enough for most things. We invest into our Rust
tooling to ensure that we can tackle new problems quickly, with a minimum of accidental complexity in
our local development environment.

> "The right tool for the job is often the tool you are already using—adding new tools has a higher
> cost than many people appreciate" — John Carmack

## The Last Stage

At the end of the day, keep trying things out, have fun, and remember—Iluvatar sees all, but the
code should be small.

> You don't really suppose, do you, that all your adventures and escapes were managed by mere luck,
> just for your sole benefit? You are a very fine person, Mr. Baggins, and I am very fond of you;
> but you are only quite a little fellow in a wide world after all!"
>
> "Thank goodness!" said Bilbo laughing, and handed him the tobacco-jar.
