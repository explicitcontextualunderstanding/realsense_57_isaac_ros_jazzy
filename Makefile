IMAGE ?= realsense_ros:debug
CACHE_DIR ?= .buildx-cache
CACHE_REF ?= realsense_ros:buildcache

.PHONY: build-cache build-cache-reg

build-cache:
	DOCKER_BUILDKIT=1 \
		docker buildx build --progress=plain \
		--cache-from type=local,src=$(CACHE_DIR) \
		--cache-to   type=local,dest=$(CACHE_DIR),mode=max \
		-t $(IMAGE) -f Dockerfile . --load

# Push/pull cache to a registry ref (requires a builder that supports registry cache)
build-cache-reg:
	DOCKER_BUILDKIT=1 \
		docker buildx build --progress=plain \
		--cache-from type=registry,ref=$(CACHE_REF) \
		--cache-to   type=registry,ref=$(CACHE_REF),mode=max \
		-t $(IMAGE) -f Dockerfile . --load

.PHONY: push
push:
	# Push the image specified by IMAGE (default: realsense_ros:debug)
	docker push $(IMAGE)

