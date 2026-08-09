# From Autoencoders to VQ-VAE: A Discovery-Driven Tutorial Series

A four-part series on learning discrete representations of MNIST digits. Each part explains the *algorithm* and the *why*, gives you exercises with concrete checkpoints, and deliberately withholds the code — the implementation is your job. Hints are provided when a detail is genuinely easy to get wrong, but they describe mechanics, not code.

**Prerequisites:** comfort with PyTorch basics (modules, optimizers, dataloaders), basic probability, and gradient descent intuition.

**The arc of the series:**

Part 1 builds a plain autoencoder and shows you what a continuous latent space looks like — and where it fails. Part 2 fixes the "holes in the latent space" problem probabilistically with a VAE. Part 3 replaces the continuous latent with a *discrete codebook* (VQ-VAE), which forces you to confront two beautiful problems: how do you backpropagate through a non-differentiable lookup (straight-through estimation), and how do you keep the codebook alive (codebook collapse). Part 4 harvests the reward: the discrete codes turn out to encode digit identity well enough that you can cluster 1s, 2s, and 3s *without ever seeing a label*.

---

## Part 1 — The Plain Autoencoder: Compression as Representation Learning

### 1.1 The idea

An autoencoder is two functions trained jointly. An encoder `E` maps an image `x` (28×28 = 784 dimensions for MNIST) to a low-dimensional latent vector `z = E(x)`, and a decoder `D` maps `z` back to an image `x̂ = D(z)`. The training objective is simply: make `x̂` look like `x`.

The interesting part is the *bottleneck*. If the latent dimension is much smaller than 784 (say, 2 to 32 dimensions), the network cannot memorize pixels — it is forced to discover the factors that matter: stroke shape, slant, thickness, and, implicitly, digit identity. Compression *is* representation learning. Everything in this series is a variation on how we constrain and structure that bottleneck.

### 1.2 The algorithm

Training is vanilla supervised learning where the input is its own target:

1. Sample a batch of images `x`.
2. Compute `z = E(x)`, then `x̂ = D(z)`.
3. Compute a reconstruction loss `L(x, x̂)`.
4. Backpropagate through decoder and encoder together; take an optimizer step.

Two sensible reconstruction losses for MNIST, which is nearly binary (ink vs. no ink):

- **MSE**: `mean((x − x̂)²)`. Treats pixels as real values. Decoder output can be unconstrained or sigmoid-squashed.
- **Binary cross-entropy per pixel**: treats each pixel as a Bernoulli probability. Requires the decoder output in (0,1), so end with a sigmoid (or use the numerically-stable "with logits" variant of the loss).

BCE typically gives sharper MNIST reconstructions because it penalizes confident wrong predictions harshly. Try both and look at the difference.

### 1.3 Architecture guidance (not code)

You can go fully-connected (784 → 256 → 64 → latent, mirrored decoder) and it will work. A convolutional version is more instructive because it sets you up for Part 3: two or three stride-2 convolutions take 28×28 down to 7×7 spatial resolution with some number of channels; the decoder mirrors this with transposed convolutions (or upsample + conv). ReLU or similar between layers; no activation (or sigmoid, per your loss choice) at the very end.

A subtle detail worth discovering yourself: 28 is not a power of two, so getting the decoder to output *exactly* 28×28 requires care with kernel sizes, padding, and output padding. Work out the arithmetic of `(H + 2·pad − kernel)/stride + 1` before you code.

### 1.4 Exercises

**Exercise 1.1 — Baseline.** Train an autoencoder with latent dimension 32. *Checkpoint:* after a few epochs on MNIST, reconstructions should be clearly recognizable digits, slightly blurry. If they are gray mush, your learning rate, loss scaling, or decoder output range is off.

**Exercise 1.2 — The 2D latent map.** Retrain with latent dimension 2, then scatter-plot the latents of the test set, colored by digit label (you may use labels for *visualization only* — that's not cheating, it's diagnosis). *Checkpoint:* you should see partially separated clouds per digit, with overlaps (4/9, 3/5/8 tend to mix). This picture is the single most important artifact of Part 1 — keep it for comparison later.

**Exercise 1.3 — The holes.** Pick two test images of different digits, take their latents `z₁, z₂`, and decode points along the straight line between them. Also decode a few *random* points sampled from roughly the same range as your latents. *Checkpoint:* interpolations often pass through non-digit garbage, and random latents frequently decode to nothing meaningful. Write down *why* you think this happens before reading Part 2.

### 1.5 What went wrong, conceptually

The autoencoder only ever learns to decode points that the encoder actually produces. Nothing constrains the *shape* of the latent distribution: it can be a thin, curved, gappy manifold with vast dead regions between the digit clouds. The latent space is a filing cabinet with no organization guarantee. Two fixes exist: force the latent distribution toward a known, well-behaved shape (Part 2), or abandon continuity entirely and use a finite set of learned symbols (Part 3).

---

## Part 2 — The Variational Autoencoder: A Probabilistic Bottleneck

### 2.1 Reframing the problem

Instead of mapping `x` to a single point, the VAE encoder outputs a *distribution* over latents — in practice, a diagonal Gaussian: a mean vector `μ(x)` and a log-variance vector `log σ²(x)` (predicting log-variance keeps σ positive and training stable). During training you *sample* `z ~ N(μ, σ²)` and decode the sample.

The training objective (the negative ELBO, if you want the derivation — worth reading once, from Kingma & Welling's "Auto-Encoding Variational Bayes") has two terms:

```
L = ReconstructionLoss(x, D(z))  +  β · KL( N(μ, σ²)  ||  N(0, I) )
```

The reconstruction term is the same as Part 1. The KL term pulls every per-image posterior toward the standard normal. The consequences: latent codes of different images overlap and pack around the origin, the space between digit clouds gets filled in, and — crucially — you can *generate* new digits by decoding `z ~ N(0, I)`.

For a diagonal Gaussian against a standard normal, the KL has a clean closed form. Derive it or look it up; it is a short expression in `μ`, `σ²`, and `log σ²`, summed over latent dimensions. Implementing it from the formula is a one-liner — the derivation is the learning.

### 2.2 The reparameterization trick — your first gradient workaround

Here is the first "how do I backprop through randomness?" problem of the series, and it foreshadows Part 3. Sampling `z ~ N(μ, σ²)` is stochastic: gradients cannot flow through a random draw into `μ` and `σ`. The trick is to move the randomness out of the path: sample `ε ~ N(0, I)` (which depends on nothing learnable), then compute

```
z = μ + σ · ε
```

Now `z` is a *deterministic, differentiable* function of `μ` and `σ`, with the noise injected as an external input. Gradients flow. Hold onto this pattern — "rewrite the computation so gradients have a path" — because the straight-through estimator in Part 3 is a blunter cousin of the same idea.

### 2.3 Exercises

**Exercise 2.1 — Convert your AE.** Change your encoder head to output `μ` and `log σ²`, add the reparameterized sampling and the KL term. *Checkpoint:* reconstructions will look slightly *worse* (blurrier) than Part 1. That is expected and worth thinking about: the KL term taxes information flow through the bottleneck.

**Exercise 2.2 — Sampling.** Decode 64 samples of `z ~ N(0, I)`. *Checkpoint:* most should be plausible (blurry) digits. Repeat Exercise 1.3's interpolation — the garbage in the middle should be substantially reduced.

**Exercise 2.3 — β sweep.** Train with β ∈ {0.1, 1, 4}. Plot the 2D latent map for each (latent dim 2 again). *Checkpoint:* small β behaves like Part 1's autoencoder; large β gives beautiful sampling but mushy reconstructions, and in the extreme the model can ignore the latent entirely (all posteriors collapse to the prior — "posterior collapse"). You are watching a rate–distortion trade-off with your own eyes.

### 2.4 Why go discrete next?

The VAE's latent is continuous and Gaussian by decree. But much of the structure in real data is naturally *discrete or symbolic* — MNIST has ten digit classes, strokes are present or absent. A continuous Gaussian bottleneck smears these symbols together (hence the blur). The VQ-VAE's bet: represent each image as a small grid of symbols drawn from a learned finite vocabulary, and let the vocabulary entries be learned vectors. Discreteness also makes downstream use (clustering in Part 4, or fitting an autoregressive prior over codes) dramatically simpler.

---

## Part 3 — VQ-VAE: Discrete Latents, Straight-Through Gradients, and Keeping the Codebook Alive

### 3.1 The architecture in one paragraph

Keep your convolutional encoder from Part 1, but stop it at the 7×7 spatial map: the encoder now outputs a grid of `7×7 = 49` continuous vectors `z_e(x)`, each of dimension `d` (e.g. d = 16 or 64). Separately, maintain a **codebook**: a learnable matrix of `K` embedding vectors `e_1 … e_K`, each also of dimension `d` (try K = 64 or 128 for MNIST). The **quantization step** replaces every one of the 49 encoder vectors with its *nearest* codebook vector in Euclidean distance:

```
k*  = argmin_k  || z_e − e_k ||²        (per spatial position)
z_q = e_{k*}
```

The decoder then reconstructs the image from the grid of quantized vectors `z_q`. So an MNIST image becomes, literally, a 7×7 grid of integers in {1…K} — a tiny "sentence" of 49 tokens over a K-word vocabulary. That grid of integers is the object Part 4 will cluster.

An implementation thought to work through yourself: computing all pairwise distances between N encoder vectors and K codebook vectors can be done without loops by expanding `||a − b||² = ||a||² − 2a·b + ||b||²` into matrix operations. Also think carefully about tensor layout — your encoder output is (batch, channels, height, width) but the distance computation wants vectors of dimension `d` in the last axis.

### 3.2 Problem one: argmin has no gradient — the straight-through estimator

(You called it "strike-through" — the standard name is **straight-through estimator**, STE, and the name describes exactly what it does.)

The quantization step is a hard nearest-neighbor lookup. Its derivative with respect to the encoder output is zero almost everywhere (moving `z_e` slightly usually doesn't change which code wins) and undefined at the boundaries. If you backpropagate naively, the encoder receives **no gradient at all** and never trains.

The straight-through estimator is an audaciously simple fix: **on the backward pass, pretend the quantizer is the identity function.** The decoder's gradient with respect to `z_q` is copied *straight through* the quantizer and handed to `z_e`, skipping the argmin entirely:

```
forward:   z_q = quantize(z_e)
backward:  ∂L/∂z_e  :=  ∂L/∂z_q      (just copy it)
```

Why is this even reasonable? Because `z_q` and `z_e` are close (that's the whole point of quantization to the *nearest* code), so a direction that improves the loss at `z_q` is usually also a good direction at `z_e`. It is a biased estimator — the bias grows with the quantization error — which is precisely why the losses in the next section work to keep `z_e` and `z_q` close together.

The classic autodiff trick to implement this in one expression, worth discovering: you want a quantity that *evaluates* to `z_q` in the forward pass but whose *gradient* flows to `z_e`. Think about what `z_e + stopgradient(z_q − z_e)` computes forward, and where its gradient goes backward. (In PyTorch, stop-gradient is `.detach()`.) Sit with that expression until it clicks — it is the crux of the whole model.

Connection to Part 2: reparameterization made gradients flow through sampling by restructuring the computation *exactly*; STE makes gradients flow through quantization by restructuring it *approximately*. One is exact, one is a useful lie.

### 3.3 Problem two: what trains the codebook?

The straight-through trick means the argmin is invisible to backprop — so the codebook vectors `e_k` receive **no gradient from the reconstruction loss either**. They need their own training signal. Two families of solutions:

**Solution A — the gradient (loss-based) approach.** Add explicit loss terms. The full VQ-VAE objective is:

```
L = ReconLoss(x, x̂)
  + || sg[z_e] − z_q ||²          (codebook loss: moves codes toward encoder outputs)
  + β · || z_e − sg[z_q] ||²      (commitment loss: moves encoder outputs toward their codes)
```

where `sg[·]` is stop-gradient. Read the stop-gradients carefully — they are the entire content of these two terms. Both terms are the *same distance*, but the first updates only the codebook (encoder side frozen by sg), pulling each chosen code toward the average of the encoder vectors that selected it; the second updates only the encoder (codebook side frozen), stopping the encoder from wandering away from the vocabulary — which would make the STE lie in 3.2 progressively worse. β ≈ 0.25 is the standard starting value. Without the commitment term, encoder outputs tend to grow without bound, chasing reconstruction gains the codebook can't follow.

**Solution B — the ad-hoc (non-gradient) approach: exponential moving averages.** Notice that the codebook loss, taken alone, has a closed-form ideal solution: each code should be the **mean of the encoder vectors currently assigned to it**. That is exactly the centroid update of k-means. So instead of gradient descent on the codebook, maintain running statistics per code and update outside the optimizer:

```
for each code k, per batch:
    n_k ← γ·n_k + (1−γ) · (count of vectors assigned to k)
    m_k ← γ·m_k + (1−γ) · (sum   of vectors assigned to k)
    e_k ← m_k / n_k
```

with decay γ ≈ 0.99. This is streaming mini-batch k-means living inside your training loop. The codebook loss term disappears from `L` (keep the commitment term!), updates are independent of the optimizer's learning rate, and in practice EMA converges faster and more stably. A numerical wrinkle to think about: what happens to `m_k / n_k` when a code hasn't been used for many batches and `n_k` decays toward zero? (The original paper's authors use Laplace smoothing on the counts; you can also just guard the division. Decide and justify.)

### 3.4 Problem three: codebook collapse

Train a VQ-VAE naively and then histogram which codes actually get used. You will very likely find that a handful of codes serve almost all positions while most of the codebook is **dead** — never selected, therefore never updated (no assignments → no gradient in Solution A, frozen statistics in Solution B), therefore never selected again. A self-reinforcing death spiral. This is *codebook collapse*, and it wastes capacity: a 128-code vocabulary that effectively uses 9 codes is a 9-code vocabulary.

The diagnostic to implement first: **perplexity** of the code usage distribution. Compute the empirical frequency `p_k` of each code over a batch (or epoch) and evaluate `exp(−Σ p_k log p_k)`. It ranges from 1 (total collapse) to K (perfectly uniform usage). Log it every epoch alongside your loss — it is the vital sign of a VQ-VAE.

Mitigations, roughly in order of bang-for-buck:

**Dead-code revival (restarts).** Periodically find codes whose usage (or EMA count `n_k`) has fallen below a threshold and re-initialize them to a randomly chosen *encoder output from the current batch*. This teleports dead vocabulary into regions of latent space where the data actually lives. Simple, ad-hoc, remarkably effective. Design questions for you: how often to check, what threshold, and why re-initializing from data beats re-initializing randomly.

**Data-driven initialization.** Collapse often starts at step zero: if random initial codes sit far from the encoder's output distribution, one lucky code captures everything. Initialize the codebook by running k-means on encoder outputs from a few warm-up batches instead of drawing random vectors.

**EMA updates themselves** (Solution B) resist collapse better than pure gradient updates, because per-code centroid updates don't compete through a shared learning rate.

**Tune the pressure.** A large commitment β herds encoder outputs tightly onto few codes; lowering it can spread usage. Too large a codebook for the data's intrinsic complexity also invites dead codes — K is a hyperparameter to sweep, not maximize.

### 3.5 Exercises

**Exercise 3.1 — Minimal VQ-VAE.** Implement quantization + STE + the three-term loss (Solution A). K = 64, d = 16, 7×7 grid. *Checkpoint:* reconstructions comparable to your plain AE and noticeably *sharper* than your VAE. If the loss trains but reconstructions stay frozen at blur, your STE wiring is broken (gradient not reaching the encoder) — this is the classic bug of this model. If loss explodes, check you didn't forget a stop-gradient.

**Exercise 3.2 — Watch a collapse.** Log perplexity. Try to *provoke* collapse: raise β to 2.0, or set K = 512, or initialize codes with large variance. *Checkpoint:* perplexity crashing to single digits while reconstruction loss still slowly improves — capacity dying quietly.

**Exercise 3.3 — EMA codebook.** Replace the codebook loss with EMA updates (Solution B). Compare against Solution A: perplexity curves, reconstruction loss, wall-clock stability across 3 seeds. *Checkpoint:* EMA typically reaches higher perplexity faster.

**Exercise 3.4 — Revival.** Add dead-code restarts to both variants. *Checkpoint:* sustained perplexity at a healthy fraction of K, and it should now be much harder to provoke collapse in the 3.2 setup.

**Exercise 3.5 — Read the codes.** For a few test images, print the 7×7 integer grid. Feed the *same digit written twice* and compare grids. *Checkpoint:* background positions share one code; similar strokes share codes across images. You are looking at a learned symbolic language for MNIST — the bridge to Part 4.

---

## Part 4 — Clustering in Code Space: Do the Symbols Know What a "3" Is?

### 4.1 The hypothesis

The VQ-VAE was trained with **zero labels**, optimizing reconstruction only. But reconstructing a digit well requires encoding its shape, and shape is what defines digit identity. Hypothesis: images of the same digit should use *similar code patterns*, so an unsupervised clustering over code representations should approximately rediscover the ten digit classes. Part 4 is about testing that hypothesis honestly.

### 4.2 Choosing a representation of an image in code space

Each image is now a 7×7 grid of integers. Integers are not vectors — code 17 is not "close to" code 18 — so you must choose how to turn a grid of symbols into something a clustering algorithm can consume. Four options, each a real design decision:

**(a) Bag-of-codes histogram.** Count how often each of the K codes appears in the image's grid; the image becomes a K-dimensional count (or frequency) vector. This is exactly bag-of-words from classical NLP: it throws away *where* codes appear and keeps *which* codes appear. Cheap, surprisingly strong, and a good default. Consider TF-IDF-style reweighting: codes that appear in every image (background!) carry no clustering information — what happens if you down-weight them?

**(b) One-hot position grid.** Keep spatial structure: represent each of the 49 positions as a one-hot over K and flatten (dimension 49·K). Preserves layout, but the distance between two grids is essentially position-wise Hamming distance — brittle to a digit being shifted by one cell. Try it and observe the failure mode.

**(c) Embedding average.** Replace each index by its codebook vector `e_k` and average (or max-pool) over the 49 positions — a d-dimensional vector. Unlike (a) and (b), this *does* respect similarity between codes: two different codes representing similar strokes are nearby vectors. This uses the geometry the codebook learned for free.

**(d) Pre-quantization pooling (baseline).** Pool the *continuous* encoder outputs `z_e` instead. If this clusters much better than (a)–(c), quantization destroyed information; if similar, the symbols captured what mattered. Always run this baseline — it tells you what the discretization cost you.

### 4.3 The clustering algorithm and the honest evaluation

The pipeline is deliberately classical — the interesting object is the representation, not the clusterer:

1. Encode the full test set (10k images) into your chosen representation.
2. Optionally L2-normalize (for histograms, cosine geometry usually beats raw Euclidean — think about why counts of different magnitudes should count the same).
3. Run k-means with k = 10. Run it with multiple restarts; k-means is init-sensitive.
4. Evaluate against the true labels — which you use **only now, only for scoring**.

Evaluation is subtle because cluster IDs are arbitrary (cluster 7 might be "the 3s"). Three standard metrics, all worth implementing:

- **Cluster purity**: assign each cluster its majority true label; purity is the fraction of images matching their cluster's majority label. Intuitive, but inflatable by making many tiny clusters.
- **Normalized Mutual Information (NMI)**: how much knowing the cluster tells you about the digit, normalized to [0,1]. Permutation-invariant by construction.
- **Adjusted Rand Index (ARI)**: over all pairs of images, do the clustering and the labels agree on "same group / different group"? Corrected for chance; 0 ≈ random, 1 = perfect.

If you want a single best-case accuracy number, find the optimal one-to-one mapping between the 10 clusters and the 10 digits — this is a linear assignment problem (the Hungarian algorithm; building the 10×10 contingency matrix and solving the assignment is a nice self-contained exercise).

*Rough expectations to calibrate against:* k-means directly on raw pixels manages ~50–60% mapped accuracy on MNIST. A decent representation from your VQ-VAE should beat that. Don't expect ~97% — reconstruction training preserves style (slant, thickness) *as well as* identity, so some clusters will split one digit into two styles, or merge 4s with 9s. That is not failure; that is the model telling you what reconstruction actually requires.

### 4.4 Exercises

**Exercise 4.1 — The bake-off.** Implement representations (a), (c), (d); cluster and score each with purity, NMI, ARI, plus the raw-pixel baseline. *Checkpoint:* a clear ranking with (d) as your upper reference. Write one paragraph explaining the ranking you observe.

**Exercise 4.2 — See it.** Run t-SNE or UMAP on your best representation, color by true digit. Compare side-by-side with your Part 1 (AE) and Part 2 (VAE) 2D latent maps. *Checkpoint:* visibly tighter, more separated digit islands than Part 1.

**Exercise 4.3 — Autopsy the confusions.** Build the 10×10 contingency matrix (clusters × digits). Find the worst merge (likely 4/9 or 3/5/8) and display 25 images from that cluster in a grid. *Checkpoint:* you should be able to articulate *visually* why reconstruction cannot tell these apart.

**Exercise 4.4 — The knobs upstream.** Does clustering improve with a larger codebook? A stronger information bottleneck (coarser 4×4 grid, or smaller d)? Sweep one factor and plot NMI against it. *Checkpoint:* an interesting non-monotonicity is common — too tight a bottleneck loses identity, too loose keeps style noise. Rate–distortion again, now measured in cluster quality.

**Exercise 4.5 (stretch) — Retrieval.** Nearest-neighbor search in code space: pick a query image, retrieve its 10 nearest test images under your best representation, display them. This is content-based image retrieval built from an unsupervised model — and a very convincing demo that the symbols mean something.

### 4.5 Where this road continues

Two natural next steps once the series is done. First, the generative direction: the 7×7 code grids are sequences over a finite vocabulary, so you can train an autoregressive prior (a PixelCNN or a small transformer) over them and sample *new* grids → decode → new digits; this two-stage recipe is the ancestor of VQ-GAN and modern discrete-token image generators. Second, the representation direction: swap MNIST for Fashion-MNIST or CIFAR-10 and rerun Part 4 to feel how the "identity vs. style" tension scales with data complexity.

---

## Suggested rhythm

One part per sitting, exercises included, keeping every diagnostic plot from earlier parts for comparison. The series is designed so each part's *failure* (holes in the AE latent, blur in the VAE, collapse in the VQ codebook) motivates the next idea — so resist the urge to fix things ahead of schedule; observe the failure first.
