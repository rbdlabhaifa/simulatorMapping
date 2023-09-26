#include <time.h>

#include "DBoW3/src/Vocabulary.h"


bool load_as_text(DBoW3::Vocabulary& voc, const std::string infile) {
	clock_t tStart = clock();
	bool res = voc.load(infile);
	printf("Loading fom text: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
	return res;
}

void load_as_xml(DBoW3::Vocabulary& voc, const std::string infile) {
	clock_t tStart = clock();
	voc.load(infile);
	printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

/*void load_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}*/

void save_as_xml(DBoW3::Vocabulary& voc, const std::string outfile) {
	clock_t tStart = clock();
	voc.save(outfile);
	printf("Saving as xml: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void save_as_text(DBoW3::Vocabulary& voc, const std::string outfile) {
	clock_t tStart = clock();
	voc.save(outfile);
	printf("Saving as text: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void save_as_binary(DBoW3::Vocabulary &voc,const std::string outfile) {
	clock_t tStart = clock();
	voc.save(outfile, true);
	printf("Saving as binary: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}


int main(int argc, char** argv) {
	std::cout << "BoW load/save benchmark" << std::endl;
	DBoW3::Vocabulary voc;
	load_as_text(voc,argv[1]);
	save_as_binary(voc,argv[2]);

	return 0;
}
