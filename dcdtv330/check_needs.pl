#!/usr/bin/perl

my @missing;

sub catcheck($)
{
  my $res = "";
  $res = qx(cat $_[0]) if (-r $_[0]);
  return $res;
}

my $system_release = catcheck("/etc/system-release");
$system_release = catcheck("/etc/redhat-release") if !$system_release;
$system_release = catcheck("/etc/lsb-release") if !$system_release;

sub give_redhat_hints
{
	my $install;

	my %map = (
		"lsdiff"		=> "patchutils",
		"Digest::SHA1"		=> "perl-Digest-SHA1",
		"Proc::ProcessTable"	=> "perl-Proc-ProcessTable",
	);

	foreach my $prog (@missing) {
		print "ERROR: please install \"$prog\", otherwise, build won't work.\n";
		if (defined($map{$prog})) {
			$install .= " " . $map{$prog};
		} else {
			$install .= " " . $prog;
		}
	}

	printf("You should run:\n\tyum install -y $install\n");
}

sub give_ubuntu_hints
{
	my $install;

	my %map = (
		"lsdiff"		=> "patchutils",
		"Digest::SHA1"		=> "libdigest-sha1-perl",
		"Proc::ProcessTable"	=> "libproc-processtable-perl",
	);

	foreach my $prog (@missing) {
		print "ERROR: please install \"$prog\", otherwise, build won't work.\n";
		if (defined($map{$prog})) {
			$install .= " " . $map{$prog};
		} else {
			$install .= " " . $prog;
		}
	}

	printf("You should run:\n\tsudo apt-get install $install\n");
}

sub give_hints
{

	# Distro-specific hints
	if ($system_release =~ /Red Hat Enterprise Linux Workstation/) {
		give_redhat_hints;
		return;
	}
	if ($system_release =~ /Fedora/) {
		give_redhat_hints;
		return;
	}
	if ($system_release =~ /Ubuntu/) {
		give_ubuntu_hints;
		return;
	}

	# Fall-back to generic hint code
	foreach my $prog (@missing) {
		print "ERROR: please install \"$prog\", otherwise, build won't work.\n";
	}
	print "I don't know distro $system_release. So, I can't provide you a hint with the package names.\n";
	print "Be welcome to contribute with a patch for media-build, by submitting a distro-specific hint\n";
	print "to linux-media\@vger.kernel.org\n";
}

my $need = 0;
sub findprog($)
{
	foreach(split(/:/, $ENV{PATH})) {
		return "$_/$_[0]" if(-x "$_/$_[0]");
	}
}

sub need_program
{
	my $prog = shift;
	my $pkgname = shift;

	return if findprog($prog);

	push @missing, $prog;

	$need++;
}

sub need_perl_module
{
	my $prog = shift;
	my $pkgname = shift;

	my $err = system("perl -M$prog -e 1 2>/dev/null /dev/null");
	return if ($err == 0);

	push @missing, $prog;

	$need++;
}

# Check for needed programs/tools
need_program "git";
need_program "make";
need_program "gcc";
need_program "patch";
need_program "lsdiff";

# Check for needed perl modules
need_perl_module "Digest::SHA1";
need_perl_module "Proc::ProcessTable";

give_hints if ($need);

die "Build can't procceed as $need dependency is missing" if ($need == 1);
die "Build can't procceed as $need dependencies are missing" if ($need);
