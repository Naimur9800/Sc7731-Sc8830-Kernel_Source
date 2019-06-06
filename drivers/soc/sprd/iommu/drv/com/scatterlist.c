#include "sprd_defs.h"
#include "scatterlist.h"
struct scatterlist *sg_next(struct scatterlist *sg)
{
	if (sg_is_last(sg))
		return NULL;

	sg++;
	if (NULL == sg_is_chain(sg))
		sg = sg_chain_ptr(sg);

	return sg;
}

