#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include <aff3ct.hpp>
using namespace aff3ct;

struct params
{
	int   K         =  32;     // number of information bits
	int   N         = 128;     // codeword size
	int   fe        = 100;     // number of frame errors
	int   seed      =   0;     // PRNG seed for the AWGN channel
	float ebn0_min  =   0.00f; // minimum SNR value
	float ebn0_max  =  10.01f; // maximum SNR value
	float ebn0_step =   1.00f; // SNR step
	float R;                   // code rate (R=K/N)
};
void init_params(params &p);

struct modules
{
	std::unique_ptr<module::Source_random<>>          source;
	std::unique_ptr<module::Encoder_repetition_sys<>> encoder;
	std::unique_ptr<module::Modem_BPSK<>>             modem;
	std::unique_ptr<module::Channel_AWGN_LLR<>>       channel;
	std::unique_ptr<module::Decoder_repetition_std<>> decoder;
	std::unique_ptr<module::Monitor_BFER<>>           monitor;
};
void init_modules(const params &p, modules &m);

struct buffers
{
	std::vector<int  > ref_bits;
	std::vector<int  > enc_bits;
	std::vector<float> symbols;
	std::vector<float> noisy_symbols;
	std::vector<float> LLRs;
	std::vector<int  > dec_bits;
	std::vector<bool > frozen_bits;//IB added
};
void init_buffers(const params &p, buffers &b);

struct utils
{
	std::unique_ptr<tools::Sigma<>>               noise;     // a sigma noise type
	std::vector<std::unique_ptr<tools::Reporter>> reporters; // list of reporters dispayed in the terminal
	std::unique_ptr<tools::Terminal_std>          terminal;  // manage the output text in the terminal
};
void init_utils(const modules &m, utils &u);

int main(int argc, char** argv)
{

	params p;  init_params (p   ); // create and initialize the parameters defined by the user
	modules m; init_modules(p, m); // create and initialize the modules
	buffers b; init_buffers(p, b); // create and initialize the buffers required by the modules
	utils u;   init_utils  (m, u); // create and initialize the utils

	// Create Frozen Bits Generator GA
	tools::Frozenbits_generator_GA         frozenBitsGeneratorGA(p.K,p.N);

	// Compute the noise (sigma)
	float SNR_max  =  10.01f;
    const auto esn0  = tools::ebn0_to_esn0 (SNR_max, p.R);
    const auto sigma = tools::esn0_to_sigma(esn0);
    u.noise->set_noise(sigma, SNR_max, esn0);

	// Set the noise for the Frozen Bits Generator GA
	frozenBitsGeneratorGA.set_noise(*u.noise);

	// Generate the frozen bits
	frozenBitsGeneratorGA.generate(b.frozen_bits);

	// Create Polar Encoder
	module::Encoder_polar<> polarEncoder(p.K, p.N, b.frozen_bits);

	// Generate Random bits
	m.source ->generate(b.ref_bits);

	// Encode the bits
	polarEncoder.encode(b.ref_bits,b.enc_bits);

	// Create Polar Decoder
	module::Decoder_polar_SC_naive<> polarDecoder(p.K, p.N, b.frozen_bits);

	// Convert the vector<int> to vector<float> such that it can be used in the decoder
	std::vector<float> encodedBits(b.enc_bits.begin(), b.enc_bits.end());

	// Decode the encoded bits
	polarDecoder.decode_siho(encodedBits,b.dec_bits);

	return 0;
}

void init_params(params &p)
{
	p.R = (float)p.K / (float)p.N;
	std::cout << "# * Simulation parameters: "              << std::endl;
	std::cout << "#    ** Frame errors   = " << p.fe        << std::endl;
	std::cout << "#    ** Noise seed     = " << p.seed      << std::endl;
	std::cout << "#    ** Info. bits (K) = " << p.K         << std::endl;
	std::cout << "#    ** Frame size (N) = " << p.N         << std::endl;
	std::cout << "#    ** Code rate  (R) = " << p.R         << std::endl;
	std::cout << "#    ** SNR min   (dB) = " << p.ebn0_min  << std::endl;
	std::cout << "#    ** SNR max   (dB) = " << p.ebn0_max  << std::endl;
	std::cout << "#    ** SNR step  (dB) = " << p.ebn0_step << std::endl;
	std::cout << "#"                                        << std::endl;
}

void init_modules(const params &p, modules &m)
{
	m.source  = std::unique_ptr<module::Source_random         <>>(new module::Source_random         <>(p.K        ));
	m.encoder = std::unique_ptr<module::Encoder_repetition_sys<>>(new module::Encoder_repetition_sys<>(p.K, p.N   ));
	m.modem   = std::unique_ptr<module::Modem_BPSK            <>>(new module::Modem_BPSK            <>(p.N        ));
	m.channel = std::unique_ptr<module::Channel_AWGN_LLR      <>>(new module::Channel_AWGN_LLR      <>(p.N, p.seed));
	m.decoder = std::unique_ptr<module::Decoder_repetition_std<>>(new module::Decoder_repetition_std<>(p.K, p.N   ));
	m.monitor = std::unique_ptr<module::Monitor_BFER          <>>(new module::Monitor_BFER          <>(p.K, p.fe  ));
};


void init_buffers(const params &p, buffers &b)
{
	b.ref_bits      = std::vector<int  >(p.K);
	b.enc_bits      = std::vector<int  >(p.N);
	b.symbols       = std::vector<float>(p.N);
	b.noisy_symbols = std::vector<float>(p.N);
	b.LLRs          = std::vector<float>(p.N);
	b.dec_bits      = std::vector<int  >(p.K);
	b.frozen_bits   = std::vector<bool >(p.N);//IB added
}

void init_utils(const modules &m, utils &u)
{
	// create a sigma noise type
	u.noise = std::unique_ptr<tools::Sigma<>>(new tools::Sigma<>());
	// report the noise values (Es/N0 and Eb/N0)
	u.reporters.push_back(std::unique_ptr<tools::Reporter>(new tools::Reporter_noise<>(*u.noise)));
	// report the bit/frame error rates
	u.reporters.push_back(std::unique_ptr<tools::Reporter>(new tools::Reporter_BFER<>(*m.monitor)));
	// report the simulation throughputs
	u.reporters.push_back(std::unique_ptr<tools::Reporter>(new tools::Reporter_throughput<>(*m.monitor)));
	// create a terminal that will display the collected data from the reporters
	u.terminal = std::unique_ptr<tools::Terminal_std>(new tools::Terminal_std(u.reporters));
}
