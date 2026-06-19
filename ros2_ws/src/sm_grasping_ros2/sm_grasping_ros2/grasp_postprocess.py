#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .gpd_wrapper import GraspCandidate


def filter_candidates(
    candidates: list[GraspCandidate],
    min_score: float,
) -> list[GraspCandidate]:
    filtered = []
    for item in candidates:
        if item.score < min_score:
            continue
        filtered.append(item)
    return sorted(filtered, key=lambda x: x.score, reverse=True)
