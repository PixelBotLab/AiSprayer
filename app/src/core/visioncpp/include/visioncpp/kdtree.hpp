#pragma once

#include "visioncpp/types.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <vector>

namespace visioncpp {

class KdTree {
public:
    void build(const std::vector<Vec3>& pts) {
        pts_ = &pts;
        idx_.resize(pts.size());
        std::iota(idx_.begin(), idx_.end(), 0);
        nodes_.clear();
        nodes_.reserve(pts.size());
        if (!pts.empty()) {
            buildRec(0, static_cast<int>(pts.size()), 0);
        }
    }

    int nearest(const Vec3& q) const {
        if (!pts_ || pts_->empty()) return -1;
        int best = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        nearestRec(0, q, best, best_d2);
        return best;
    }

    void knn(const Vec3& q, int k, std::vector<int>& out, std::vector<double>* d2 = nullptr) const {
        out.clear();
        if (d2) d2->clear();
        if (!pts_ || pts_->empty() || k <= 0) return;
        using Pair = std::pair<double, int>;
        std::priority_queue<Pair> heap;
        knnRec(0, q, k, heap);
        out.resize(heap.size());
        if (d2) d2->resize(heap.size());
        for (int i = static_cast<int>(heap.size()) - 1; i >= 0; --i) {
            out[i] = heap.top().second;
            if (d2) (*d2)[i] = heap.top().first;
            heap.pop();
        }
    }

    void radius(const Vec3& q, double r, int max_nn, std::vector<int>& out) const {
        out.clear();
        if (!pts_ || pts_->empty() || r <= 0) return;
        const double r2 = r * r;
        radiusRec(0, q, r2, max_nn, out);
    }

    size_t size() const { return pts_ ? pts_->size() : 0; }

private:
    struct Node {
        int start = 0, end = 0, axis = 0, mid = -1;
        int left = -1, right = -1;
    };

    const std::vector<Vec3>* pts_ = nullptr;
    std::vector<int> idx_;
    std::vector<Node> nodes_;

    int buildRec(int start, int end, int depth) {
        const int id = static_cast<int>(nodes_.size());
        nodes_.push_back({});
        nodes_[id].start = start;
        nodes_[id].end = end;
        nodes_[id].axis = depth % 3;
        const int count = end - start;
        if (count <= 0) return id;
        if (count == 1) {
            nodes_[id].mid = idx_[start];
            return id;
        }
        const int mid = start + count / 2;
        const int axis = nodes_[id].axis;
        std::nth_element(idx_.begin() + start, idx_.begin() + mid, idx_.begin() + end,
                         [&](int a, int b) { return (*pts_)[a][axis] < (*pts_)[b][axis]; });
        nodes_[id].mid = idx_[mid];
        if (mid > start) nodes_[id].left = buildRec(start, mid, depth + 1);
        if (mid + 1 < end) nodes_[id].right = buildRec(mid + 1, end, depth + 1);
        return id;
    }

    void nearestRec(int node, const Vec3& q, int& best, double& best_d2) const {
        if (node < 0) return;
        const Node& n = nodes_[node];
        if (n.mid < 0) return;
        const Vec3& p = (*pts_)[n.mid];
        const double d2 = (p - q).squaredNorm();
        if (d2 < best_d2) {
            best_d2 = d2;
            best = n.mid;
        }
        const double delta = q[n.axis] - p[n.axis];
        const int first = delta <= 0 ? n.left : n.right;
        const int second = delta <= 0 ? n.right : n.left;
        nearestRec(first, q, best, best_d2);
        if (delta * delta < best_d2) nearestRec(second, q, best, best_d2);
    }

    void knnRec(int node, const Vec3& q, int k, std::priority_queue<std::pair<double, int>>& heap) const {
        if (node < 0) return;
        const Node& n = nodes_[node];
        if (n.mid < 0) return;
        const Vec3& p = (*pts_)[n.mid];
        const double d2 = (p - q).squaredNorm();
        if (static_cast<int>(heap.size()) < k) {
            heap.emplace(d2, n.mid);
        } else if (d2 < heap.top().first) {
            heap.pop();
            heap.emplace(d2, n.mid);
        }
        const double delta = q[n.axis] - p[n.axis];
        const int first = delta <= 0 ? n.left : n.right;
        const int second = delta <= 0 ? n.right : n.left;
        knnRec(first, q, k, heap);
        const double worst = static_cast<int>(heap.size()) < k ? std::numeric_limits<double>::infinity()
                                                              : heap.top().first;
        if (delta * delta < worst) knnRec(second, q, k, heap);
    }

    void radiusRec(int node, const Vec3& q, double r2, int max_nn, std::vector<int>& out) const {
        if (node < 0 || static_cast<int>(out.size()) >= max_nn) return;
        const Node& n = nodes_[node];
        if (n.mid < 0) return;
        const Vec3& p = (*pts_)[n.mid];
        if ((p - q).squaredNorm() <= r2) out.push_back(n.mid);
        const double delta = q[n.axis] - p[n.axis];
        const int first = delta <= 0 ? n.left : n.right;
        const int second = delta <= 0 ? n.right : n.left;
        radiusRec(first, q, r2, max_nn, out);
        if (delta * delta <= r2) radiusRec(second, q, r2, max_nn, out);
    }
};

}  // namespace visioncpp
